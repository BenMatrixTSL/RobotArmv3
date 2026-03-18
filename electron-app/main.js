/**
 * Electron Main Process
 * 
 * This is the main entry point for the Electron application.
 * It creates the application window and handles system events.
 */

const { app, BrowserWindow, screen, ipcMain } = require('electron');
const path = require('path');

// Keep a global reference of the window object
let mainWindow;
let visualisationWindow = null;

/**
 * Creates the main application window
 */
function createWindow() {
    // Get primary display size
    const primaryDisplay = screen.getPrimaryDisplay();
    const { width: screenWidth, height: screenHeight } = primaryDisplay.workAreaSize;
    
    // Detect if this is a small display (like 800x480)
    const isSmallDisplay = screenWidth <= 800 || screenHeight <= 600;
    
    // Set window size based on display
    let windowWidth = 1200;
    let windowHeight = 800;
    
    if (isSmallDisplay) {
        // For small displays, use full screen or nearly full screen
        windowWidth = Math.min(screenWidth - 20, 800);
        windowHeight = Math.min(screenHeight - 20, 600);
    } else {
        // For larger displays, use default size but don't exceed screen
        windowWidth = Math.min(1200, screenWidth - 40);
        windowHeight = Math.min(800, screenHeight - 40);
    }
    
    // Create the browser window
    mainWindow = new BrowserWindow({
        width: windowWidth,
        height: windowHeight,
        minWidth: 400,
        minHeight: 300,
        webPreferences: {
            // Enable Node.js in the renderer so we can keep things simple
            nodeIntegration: true,
            contextIsolation: false,
            // Keep preload for backwards compatibility (safe even with nodeIntegration on)
            preload: path.join(__dirname, 'preload.js')
        }
    });
    
    // Center window on screen
    mainWindow.center();
    
    // For small displays, maximize the window
    if (isSmallDisplay) {
        mainWindow.maximize();
    }

    // Load the HTML file
    // Use loadFile for proper path resolution in Electron
    mainWindow.loadFile(path.join(__dirname, 'index.html'));

    // Open DevTools in development mode (comment out in production)
    // mainWindow.webContents.openDevTools();

    // Handle window closed
    mainWindow.on('closed', function () {
        // Dereference the window object
        mainWindow = null;
    });
}

// This method will be called when Electron has finished initialization
app.whenReady().then(() => {
    createWindow();

    // On macOS, re-create window when dock icon is clicked
    app.on('activate', function () {
        if (BrowserWindow.getAllWindows().length === 0) {
            createWindow();
        }
    });
});

// Handle request from renderer to open a separate visualisation window
ipcMain.handle('open-visualisation-window', () => {
    // If it already exists, just focus it
    if (visualisationWindow && !visualisationWindow.isDestroyed()) {
        visualisationWindow.focus();
        return;
    }

    // Create a new always-on-top window for the 3D visualisation
    visualisationWindow = new BrowserWindow({
        width: 800,
        height: 600,
        minWidth: 400,
        minHeight: 300,
        alwaysOnTop: true,
        title: 'Robot Arm 3D Visualisation',
        webPreferences: {
            nodeIntegration: true,
            contextIsolation: false,
            preload: path.join(__dirname, 'preload.js')
        }
    });

    visualisationWindow.center();

    // Load the same HTML but with a query parameter so the renderer
    // knows to switch to the visualisation tab automatically.
    visualisationWindow.loadFile(path.join(__dirname, 'index.html'), {
        search: '?visualisationOnly=1'
    });

    visualisationWindow.on('closed', () => {
        visualisationWindow = null;
    });
});

// Quit when all windows are closed (except on macOS)
app.on('window-all-closed', function () {
    // On macOS, keep app running even when all windows are closed
    if (process.platform !== 'darwin') {
        app.quit();
    }
});

