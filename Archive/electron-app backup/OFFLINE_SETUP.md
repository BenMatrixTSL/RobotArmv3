# Offline Setup Instructions

This application is designed to work **completely offline** after initial setup. All external libraries are bundled locally.

## Initial Setup (One-time, requires internet)

1. **Install Dependencies**
   ```bash
   cd electron-app
   npm install
   ```
   This installs:
   - `blockly` - Visual programming library
   - `three` - 3D graphics library
   - `electron` - Desktop application framework
   - `ws` - WebSocket library (for Node.js server)

2. **Copy Libraries for Offline Use**
   ```bash
   npm run setup-libs
   ```
   This copies Blockly and Three.js from `node_modules` to the `libs/` directory.

   **Note:** The `setup-libs` script runs automatically after `npm install` (via `postinstall` hook).

## Running the Application

After setup, the application works completely offline:

```bash
npm start
```

## How Offline Mode Works

1. **Local Libraries**: Blockly and Three.js are copied to `libs/` directory
2. **Fallback System**: If local files aren't found, the app tries CDN (requires internet)
3. **No Internet Required**: Once libraries are in `libs/`, the app works offline

## File Structure

```
electron-app/
├── libs/                    # Local libraries (created by setup-libs)
│   ├── three.min.js
│   ├── blockly.min.js
│   └── javascript_compressed.js
├── node_modules/            # npm dependencies
├── index.html              # Main HTML file
├── app.js                  # Main application logic
├── robotArmClient.js       # WebSocket client
├── gcodeProcessor.js       # G-code parser
├── kinematics.js           # Forward/inverse kinematics
├── robotArm3D.js           # 3D visualization
├── blocklyRobotArm.js      # Blockly integration
├── positionsManager.js     # Stored positions manager
└── package.json            # Dependencies and scripts
```

## Troubleshooting

**Problem**: Libraries not loading, app shows CDN errors

**Solution**: 
1. Make sure `npm install` completed successfully
2. Run `npm run setup-libs` manually
3. Check that `libs/` directory exists and contains the three files
4. If files are missing, check `node_modules/` for installed packages

**Problem**: App works but shows warnings about CDN fallback

**Solution**: This is normal if local files aren't found. The app will use CDN as fallback. For true offline use, ensure `libs/` directory has all files.

## Development vs Production

- **Development**: Can use CDN fallback if local files aren't set up
- **Production**: Should always have local files in `libs/` for offline operation

## Notes

- The `libs/` directory is created automatically and should be included in version control
- If you update dependencies, run `npm run setup-libs` again to update local copies
- The setup script is idempotent - safe to run multiple times







