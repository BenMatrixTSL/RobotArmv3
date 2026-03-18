# Robot Arm Control Application

A comprehensive Electron-based desktop application for controlling a robot arm via Raspberry Pi. Features include pendant control, G-code processing, 3D visualization, visual programming (Blockly), and stored position management.

## Features

- **Pendant Control**: Real-time joint control with angle display and movement controls
- **G-Code Processing**: Load and execute G-code files for automated movements
- **3D Visualization**: Real-time 3D visualization of the robot arm using Three.js
- **Visual Programming**: Drag-and-drop Blockly interface for creating control programs
- **Stored Positions**: Save and recall up to 100 preset positions (0-99) with custom labels
- **Kinematics**: Forward and inverse kinematics calculations
- **AS5600 Calibration**: Automated sensor calibration routine
- **Offline Capable**: All libraries bundled locally - works without internet connection

## Installation

### Prerequisites

- Node.js (v14 or higher)
- npm (comes with Node.js)

### Setup Steps

1. **Install Dependencies**
   ```bash
   cd electron-app
   npm install
   ```
   This installs all required packages including Blockly, Three.js, and Electron.

2. **Setup Offline Libraries** (Automatic)
   ```bash
   npm run setup-libs
   ```
   This copies Blockly and Three.js to the `libs/` directory for offline use.
   
   **Note:** This runs automatically after `npm install` via the `postinstall` script.

3. **Run the Application**
   ```bash
   npm start
   ```

## Offline Operation

The application is designed to work **completely offline** after initial setup. See [OFFLINE_SETUP.md](./OFFLINE_SETUP.md) for detailed information.

**Key Points:**
- All external libraries (Blockly, Three.js) are bundled locally in the `libs/` directory
- No internet connection required after initial `npm install`
- CDN fallback available if local files aren't found (requires internet)

## Project Structure

```
electron-app/
├── libs/                    # Local libraries (offline mode)
│   ├── three.min.js
│   ├── blockly.min.js
│   └── javascript_compressed.js
├── index.html              # Main HTML interface
├── app.js                  # Main application logic and UI handlers
├── robotArmClient.js       # WebSocket client for Raspberry Pi communication
├── gcodeProcessor.js       # G-code file parser and executor
├── kinematics.js           # Forward/inverse kinematics calculations
├── robotArm3D.js           # Three.js-based 3D visualization
├── blocklyRobotArm.js      # Blockly visual programming integration
├── positionsManager.js     # Stored positions management (0-99)
├── main.js                 # Electron main process
├── preload.js              # Electron preload script
├── styles.css              # Application styling
└── package.json            # Dependencies and scripts
```

## Usage

### Connecting to Raspberry Pi

1. Enter the Raspberry Pi IP address (default: 192.168.1.100)
2. Enter the port (default: 8080)
3. Click "Connect" or use "Auto-Detect Pi" to find it on the network
4. Connection status is shown in the header

### Pendant Control

- Use the joint controls to move individual joints
- View real-time angle feedback
- Use quick movement buttons for preset angles
- Monitor joint status (angle, moving state, etc.)

### G-Code Control

1. Click "Choose File" to select a G-code file
2. Review the parsed commands
3. Click "Run" to execute the G-code program
4. Monitor progress in real-time

### Visual Programming (Blockly)

1. Open the "Visual Programming" tab
2. Drag blocks from the toolbox to create programs
3. Connect blocks together to build sequences
4. Click "Run Program" to execute
5. Save/load programs for reuse

### Stored Positions

1. Open the "Stored Positions" tab
2. Enter a position number (0-99) and label
3. Set joint angles manually or click "Load from Robot"
4. Click "Save Position" to store
5. Use positions in Blockly programs via "Move to Position" block

### 3D Visualization

1. Load joint configurations from Settings
2. Open the "3D Visualization" tab
3. View real-time robot arm position
4. Use mouse to rotate/zoom the view
5. Toggle between real robot angles and simulated angles

## Configuration

### Settings Tab

- **Number of Joints**: Set how many joints your robot arm has (1-6)
- **Update Interval**: How often to poll robot status (ms)
- **Default Step Size**: Default angle increment for movements (degrees)
- **Kinematics Configuration**: Load DH parameters for forward/inverse kinematics
- **AS5600 Calibration**: Calibrate joint sensors

## Development

### Running in Development Mode

```bash
npm run dev
```

This opens the application with developer tools enabled.

### Adding New Features

The codebase is well-commented and organized:
- Each module has a clear purpose
- Functions are documented with JSDoc comments
- Code follows a simple, beginner-friendly style

### Building for Distribution

Use Electron Builder or similar tools to package the application for distribution.

## Troubleshooting

### Libraries Not Loading

**Problem**: Blockly or Three.js fail to load

**Solution**:
1. Run `npm install` to ensure dependencies are installed
2. Run `npm run setup-libs` to copy libraries locally
3. Check that `libs/` directory contains the required files
4. Check browser console (F12) for specific errors

### Connection Issues

**Problem**: Cannot connect to Raspberry Pi

**Solution**:
1. Verify Raspberry Pi is running the server
2. Check IP address and port are correct
3. Ensure both devices are on the same network
4. Try "Auto-Detect Pi" feature
5. Check firewall settings

### 3D Visualization Not Showing

**Problem**: 3D view is blank or shows errors

**Solution**:
1. Load joint configurations from Settings tab first
2. Ensure Three.js library loaded successfully
3. Check browser console for errors
4. Try refreshing the visualization tab

## Dependencies

- **electron**: Desktop application framework
- **blockly**: Visual programming library
- **three**: 3D graphics library
- **ws**: WebSocket library (for Node.js server on Raspberry Pi)

## License

MIT License - See LICENSE file for details

## Support

For issues, questions, or contributions, please refer to the main project repository.

## Credits

- Built with Electron
- 3D visualization powered by Three.js
- Visual programming powered by Blockly
- Robot arm control via WebSocket communication
