/**
 * Setup Script for Offline Libraries
 * 
 * This script copies Blockly and Three.js libraries from node_modules
 * to a local libs/ directory so the app can work offline.
 * 
 * Run this script after npm install, or it will run automatically via postinstall.
 */

const fs = require('fs');
const path = require('path');

/**
 * Copies a file from source to destination, creating directories as needed
 */
function copyFile(source, dest) {
    try {
        // Create destination directory if it doesn't exist
        const destDir = path.dirname(dest);
        if (!fs.existsSync(destDir)) {
            fs.mkdirSync(destDir, { recursive: true });
        }
        
        // Copy the file
        fs.copyFileSync(source, dest);
        console.log(`✓ Copied: ${path.basename(dest)}`);
        return true;
    } catch (error) {
        console.error(`✗ Failed to copy ${source}:`, error.message);
        return false;
    }
}

/**
 * Main setup function
 */
function setupLibs() {
    console.log('Setting up offline libraries...\n');
    
    const libsDir = path.join(__dirname, 'libs');
    const nodeModulesDir = path.join(__dirname, 'node_modules');
    
    // Create libs directory
    if (!fs.existsSync(libsDir)) {
        fs.mkdirSync(libsDir, { recursive: true });
    }
    
    let successCount = 0;
    let failCount = 0;
    
    // Copy Three.js
    const threeSource = path.join(nodeModulesDir, 'three', 'build', 'three.min.js');
    const threeDest = path.join(libsDir, 'three.min.js');
    if (fs.existsSync(threeSource)) {
        if (copyFile(threeSource, threeDest)) {
            successCount++;
        } else {
            failCount++;
        }
    } else {
        console.warn('⚠ Three.js not found in node_modules. Run "npm install" first.');
        failCount++;
    }
    
    // Copy Blockly main library
    const blocklySource = path.join(nodeModulesDir, 'blockly', 'blockly.min.js');
    const blocklyDest = path.join(libsDir, 'blockly.min.js');
    if (fs.existsSync(blocklySource)) {
        if (copyFile(blocklySource, blocklyDest)) {
            successCount++;
        } else {
            failCount++;
        }
    } else {
        console.warn('⚠ Blockly not found in node_modules. Run "npm install" first.');
        failCount++;
    }
    
    // Copy Blockly JavaScript generator
    const blocklyJsSource = path.join(nodeModulesDir, 'blockly', 'javascript_compressed.js');
    const blocklyJsDest = path.join(libsDir, 'javascript_compressed.js');
    if (fs.existsSync(blocklyJsSource)) {
        if (copyFile(blocklyJsSource, blocklyJsDest)) {
            successCount++;
        } else {
            failCount++;
        }
    } else {
        console.warn('⚠ Blockly JavaScript generator not found in node_modules.');
        failCount++;
    }
    
    // Copy Blockly XML module (needed for save/load functionality)
    const blocklyXmlSource = path.join(nodeModulesDir, 'blockly', 'xml_compressed.js');
    const blocklyXmlDest = path.join(libsDir, 'xml_compressed.js');
    if (fs.existsSync(blocklyXmlSource)) {
        if (copyFile(blocklyXmlSource, blocklyXmlDest)) {
            successCount++;
        } else {
            failCount++;
        }
    } else {
        console.warn('⚠ Blockly XML module not found in node_modules.');
        failCount++;
    }
    
    console.log(`\nSetup complete: ${successCount} files copied, ${failCount} failed`);
    
    if (failCount > 0) {
        console.log('\n⚠ Some files failed to copy. The app will try to load from CDN as fallback.');
        console.log('Make sure to run "npm install" first to install dependencies.');
    } else {
        console.log('\n✓ All libraries copied successfully. App is ready for offline use.');
    }
}

// Run setup
setupLibs();


