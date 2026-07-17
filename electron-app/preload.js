/**
 * Preload Script
 *
 * This script runs in the renderer process before the page loads.
 * It provides a secure bridge between the renderer and Node.js APIs.
 */

const { contextBridge, ipcRenderer } = require('electron');
const fs = require('fs');
const path = require('path');

const api = {
    // Basic app info
    platform: process.platform,
    version: process.versions.electron,

    /**
     * Reads a text file from the app directory (synchronously).
     * Tries several path bases so the file is found whether run from electron-app or parent folder.
     * @param {string} relativePath - Path relative to the app (e.g. 'kinematics.urdf')
     * @returns {string|null} File contents, or null if read fails
     */
    readTextFile: (relativePath) => {
        const bases = [
            __dirname,
            process.cwd(),
            path.join(process.cwd(), 'electron-app')
        ];
        for (let i = 0; i < bases.length; i++) {
            const fullPath = path.join(bases[i], relativePath);
            try {
                if (fs.existsSync(fullPath)) {
                    const content = fs.readFileSync(fullPath, 'utf8');
                    if (i > 0) {
                        console.log('[preload] readTextFile: loaded from', fullPath);
                    }
                    return content;
                }
            } catch (e) {
                // try next base
            }
        }
        console.error('[preload] readTextFile: not found for', relativePath, 'tried:', bases.map(b => path.join(b, relativePath)));
        return null;
    },

    /**
     * Asks the main process to open a separate always-on-top window
     * that focuses on the 3D visualisation tab.
     */
    openVisualisationWindow: () => {
        try {
            ipcRenderer.invoke('open-visualisation-window');
        } catch (e) {
            console.error('[preload] openVisualisationWindow failed:', e);
        }
    },

    /**
     * Ask the main process to update one of the local git repositories.
     * Valid values for "which" are:
     *  - 'electron'  → update the electron-app folder
     */
    updateFromGit: async (which) => {
        try {
            return await ipcRenderer.invoke('update-from-git', { which });
        } catch (e) {
            console.error('[preload] updateFromGit failed:', e);
            return { ok: false, message: e.message || String(e) };
        }
    }
};

// contextBridge.exposeInMainWorld only works when contextIsolation is enabled.
// When contextIsolation is false (nodeIntegration mode), set window directly.
try {
    contextBridge.exposeInMainWorld('electronAPI', api);
} catch (e) {
    // contextIsolation is disabled — preload shares the renderer's global scope
    window.electronAPI = api;
}
