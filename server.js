const express = require('express');
const fs = require('fs');
const path = require('path');
const https = require('https');
const app = express();
const port = process.env.PORT || 80;

// ==================== SETUP DIRECTORIES ====================
const uploadsDir = path.join(__dirname, 'uploads');
const publicDir = path.join(__dirname, 'public');

// Tạo thư mục nếu chưa có
[uploadsDir, publicDir].forEach(dir => {
  if (!fs.existsSync(dir)) {
    fs.mkdirSync(dir, { recursive: true });
    console.log(`✅ Created directory: ${dir}`);
  }
});

// ==================== MIDDLEWARE ====================

// CORS Middleware - Cho phép tất cả origins
app.use((req, res, next) => {
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type, Authorization, ngrok-skip-browser-warning');
  res.setHeader('Access-Control-Max-Age', '86400'); // Cache preflight 24h
  
  // Handle preflight
  if (req.method === 'OPTIONS') {
    return res.sendStatus(204);
  }
  next();
});

// Request logger
app.use((req, res, next) => {
  console.log(`[${new Date().toISOString()}] ${req.method} ${req.path}`);
  next();
});

// Static files
app.use(express.static('public'));
app.use('/uploads', express.static('uploads'));

// Body parsers
app.use(express.json());
app.use(express.urlencoded({ extended: true }));
app.use(express.raw({ type: 'image/jpeg', limit: '10mb' }));

// ==================== ROUTES ====================

// Health check
app.get('/health', (req, res) => {
  res.json({ 
    status: 'healthy',
    timestamp: new Date().toISOString(),
    uptime: process.uptime()
  });
});

// Root endpoint
app.get('/', (req, res) => {
  res.json({
    service: 'Landslide Monitoring API',
    version: '1.0.0',
    endpoints: {
      upload: 'POST /upload',
      photos: 'GET /api/photos',
      deletePhoto: 'DELETE /api/photos/:filename',
      proxy: 'GET /proxy/external-photos',
      health: 'GET /health'
    }
  });
});

// Upload ảnh từ ESP32-CAM
app.post('/upload', (req, res) => {
  console.log('📸 Receiving photo from ESP32-CAM...');
  
  if (!req.body || req.body.length === 0) {
    console.error('❌ No image data received');
    return res.status(400).json({ 
      success: false,
      error: 'No image data received' 
    });
  }

  const timestamp = Date.now();
  const filename = `photo_${timestamp}.jpg`;
  const filepath = path.join(uploadsDir, filename);

  fs.writeFile(filepath, req.body, (err) => {
    if (err) {
      console.error('❌ Error saving file:', err);
      return res.status(500).json({ 
        success: false,
        error: 'Failed to save image',
        details: err.message 
      });
    }

    console.log(`✅ Photo saved: ${filename} (${req.body.length} bytes)`);
    
    res.json({ 
      success: true, 
      message: 'Photo uploaded successfully',
      filename: filename,
      url: `/uploads/${filename}`,
      size: req.body.length,
      timestamp: timestamp
    });
  });
});

// Lấy danh sách ảnh
app.get('/api/photos', (req, res) => {
  console.log('📋 Fetching photo list...');
  
  fs.readdir(uploadsDir, (err, files) => {
    if (err) {
      console.error('❌ Failed to read directory:', err);
      return res.status(500).json({ 
        success: false,
        error: 'Failed to read directory',
        details: err.message 
      });
    }

    const photos = files
      .filter(file => file.endsWith('.jpg') || file.endsWith('.jpeg'))
      .map(file => {
        try {
          const stats = fs.statSync(path.join(uploadsDir, file));
          return {
            filename: file,
            url: `/uploads/${file}`,
            timestamp: stats.mtime.getTime(),
            size: stats.size,
            created: stats.birthtime
          };
        } catch (error) {
          console.error(`⚠️ Error reading file ${file}:`, error);
          return null;
        }
      })
      .filter(photo => photo !== null)
      .sort((a, b) => b.timestamp - a.timestamp);

    console.log(`✅ Found ${photos.length} photos`);
    res.json(photos);
  });
});

// Lấy một ảnh cụ thể
app.get('/api/photos/:filename', (req, res) => {
  const filename = req.params.filename;
  const filepath = path.join(uploadsDir, filename);

  if (!fs.existsSync(filepath)) {
    return res.status(404).json({ 
      success: false,
      error: 'Photo not found' 
    });
  }

  const stats = fs.statSync(filepath);
  res.json({
    success: true,
    photo: {
      filename: filename,
      url: `/uploads/${filename}`,
      timestamp: stats.mtime.getTime(),
      size: stats.size,
      created: stats.birthtime
    }
  });
});

// Xóa ảnh
app.delete('/api/photos/:filename', (req, res) => {
  const filename = req.params.filename;
  const filepath = path.join(uploadsDir, filename);

  console.log(`🗑️ Deleting photo: ${filename}`);

  // Kiểm tra file có tồn tại không
  if (!fs.existsSync(filepath)) {
    console.error(`❌ File not found: ${filename}`);
    return res.status(404).json({ 
      success: false,
      error: 'Photo not found' 
    });
  }

  fs.unlink(filepath, (err) => {
    if (err) {
      console.error('❌ Failed to delete photo:', err);
      return res.status(500).json({ 
        success: false,
        error: 'Failed to delete photo',
        details: err.message 
      });
    }

    console.log(`✅ Photo deleted: ${filename}`);
    res.json({ 
      success: true, 
      message: 'Photo deleted successfully',
      filename: filename 
    });
  });
});

// Xóa tất cả ảnh
app.delete('/api/photos', (req, res) => {
  console.log('🗑️ Deleting all photos...');

  fs.readdir(uploadsDir, (err, files) => {
    if (err) {
      console.error('❌ Failed to read directory:', err);
      return res.status(500).json({ 
        success: false,
        error: 'Failed to read directory' 
      });
    }

    const photos = files.filter(file => file.endsWith('.jpg') || file.endsWith('.jpeg'));
    let deleted = 0;
    let errors = 0;

    photos.forEach(file => {
      try {
        fs.unlinkSync(path.join(uploadsDir, file));
        deleted++;
      } catch (error) {
        console.error(`⚠️ Failed to delete ${file}:`, error);
        errors++;
      }
    });

    console.log(`✅ Deleted ${deleted} photos, ${errors} errors`);
    res.json({ 
      success: true,
      message: 'Bulk delete completed',
      deleted: deleted,
      errors: errors 
    });
  });
});

// Proxy cho external photos (bypass CORS)
app.get('/proxy/external-photos', (req, res) => {
  const target = 'https://loading-porter-higher-wines.trycloudflare.com/api/photos';
  console.log(`🔄 Proxying request to: ${target}`);

  const proxyReq = https.get(target, (proxyRes) => {
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.statusCode = proxyRes.statusCode || 200;

    // Copy headers
    for (const [key, value] of Object.entries(proxyRes.headers || {})) {
      if (key.toLowerCase() !== 'set-cookie') {
        res.setHeader(key, value);
      }
    }

    proxyRes.pipe(res);
  });

  proxyReq.on('error', (err) => {
    console.error('❌ Error proxying external photos:', err);
    res.status(502).json({ 
      success: false,
      error: 'Failed to fetch external resource',
      details: err.message 
    });
  });

  proxyReq.end();
});

// Thống kê
app.get('/api/stats', (req, res) => {
  fs.readdir(uploadsDir, (err, files) => {
    if (err) {
      return res.status(500).json({ error: 'Failed to read directory' });
    }

    const photos = files.filter(file => file.endsWith('.jpg') || file.endsWith('.jpeg'));
    let totalSize = 0;

    photos.forEach(file => {
      try {
        const stats = fs.statSync(path.join(uploadsDir, file));
        totalSize += stats.size;
      } catch (error) {
        console.error(`Error reading file ${file}:`, error);
      }
    });

    res.json({
      success: true,
      stats: {
        totalPhotos: photos.length,
        totalSize: totalSize,
        totalSizeMB: (totalSize / (1024 * 1024)).toFixed(2),
        averageSize: photos.length > 0 ? Math.round(totalSize / photos.length) : 0,
        uploadsDirectory: uploadsDir
      }
    });
  });
});

// ==================== ERROR HANDLING ====================

// 404 handler
app.use((req, res) => {
  res.status(404).json({ 
    success: false,
    error: 'Endpoint not found',
    path: req.path,
    method: req.method
  });
});

// Global error handler
app.use((err, req, res, next) => {
  console.error('❌ Global error:', err);
  res.status(500).json({ 
    success: false,
    error: 'Internal server error',
    message: err.message,
    stack: process.env.NODE_ENV === 'development' ? err.stack : undefined
  });
});

// ==================== START SERVER ====================

const server = app.listen(port, () => {
  console.log('\n' + '='.repeat(60));
  console.log('🚀 LANDSLIDE MONITORING SERVER');
  console.log('='.repeat(60));
  console.log(`📡 Server running on: http://localhost:${port}`);
  console.log(`📁 Uploads directory: ${uploadsDir}`);
  console.log(`📂 Public directory: ${publicDir}`);
  console.log('\n📋 Available Endpoints:');
  console.log(`   POST   /upload                    - Upload photo`);
  console.log(`   GET    /api/photos                - List all photos`);
  console.log(`   GET    /api/photos/:filename      - Get photo info`);
  console.log(`   DELETE /api/photos/:filename      - Delete photo`);
  console.log(`   DELETE /api/photos                - Delete all photos`);
  console.log(`   GET    /api/stats                 - Get statistics`);
  console.log(`   GET    /proxy/external-photos     - Proxy external API`);
  console.log(`   GET    /health                    - Health check`);
  console.log('='.repeat(60) + '\n');
});

// Graceful shutdown
process.on('SIGTERM', () => {
  console.log('⏹️ SIGTERM received, closing server...');
  server.close(() => {
    console.log('✅ Server closed');
    process.exit(0);
  });
});

process.on('SIGINT', () => {
  console.log('\n⏹️ SIGINT received, closing server...');
  server.close(() => {
    console.log('✅ Server closed');
    process.exit(0);
  });
});

module.exports = app;