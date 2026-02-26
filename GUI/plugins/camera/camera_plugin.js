//plugins/camera/camera_plugin.js

// Define the key for your new object type
// Renamed from IPHONE_CAMERA_KEY to CAMERA_KEY
const CAMERA_KEY = 'camera';

// Define the plugin factory function
// Renamed from iPhoneCameraPlugin to CameraPlugin
window.CameraPlugin = function CameraPlugin() {
    return function install(openmct) {
        // --- Define the new object type ---
        // Renamed from iPhone Camera to Camera
        // Updated key to CAMERA_KEY
        openmct.types.addType(CAMERA_KEY, {
            name: 'Camera',
            description: 'Displays a video feed from a camera stream URL.',
            creatable: true,
            cssClass: 'icon-camera', // Using a camera icon
            initialize(domainObject) {
                // Initialize properties for the object
                // The camera feed URL will be stored here
                domainObject.cameraFeedUrl = '';
            },
            form: [
                {
                    key: 'cameraFeedUrl',
                    name: 'Camera Feed URL (e.g., MJPEG stream URL)',
                    control: 'textfield',
                    required: true,
                    cssClass: 'l-input'
                }
            ]
        });
        // --- End Define new object type ---

        // --- Define the view provider for the new object type ---
        // Updated key to camera-view
        // Updated canView to check for CAMERA_KEY
        // Renamed name to Camera View
        openmct.objectViews.addProvider({
            key: 'camera-view',
            name: 'Camera View',
            canView: (domainObject) => {
                // This view can display objects of type 'camera'
                return domainObject.type === CAMERA_KEY;
            },
            view: (domainObject) => {
                let cameraElement = null;

                return {
                    show(element) {
                        // Create an image element to display the stream
                        cameraElement = document.createElement('img');
                        // Set initial styles for responsiveness
                        cameraElement.style.width = '100%';
                        cameraElement.style.height = '100%';
                        cameraElement.style.objectFit = 'contain'; // Ensure the image fits without distortion

                        // Set the source URL from the domain object property
                        // If the camera provides an MJPEG stream, setting the src
                        // attribute to the stream URL will often display the live feed directly.
                        // If it provides a static image, the browser might just show the latest
                        // image, or you might need JavaScript to refresh the src periodically.
                        // For simplicity, we'll start with just setting the src.
                        if (domainObject.cameraFeedUrl) {
                            cameraElement.src = domainObject.cameraFeedUrl;
                             console.log(`Camera Plugin: Attempting to load feed from: ${domainObject.cameraFeedUrl}`);
                        } else {
                             // Display a message if the URL is not set
                             const messageElement = document.createElement('div');
                             messageElement.style.textAlign = 'center';
                             messageElement.style.marginTop = '20px';
                             messageElement.textContent = 'Camera Feed URL not configured.';
                             element.appendChild(messageElement);
                             console.warn('Camera Plugin: Camera Feed URL is not set.');
                        }


                        // Append the image element to the provided parent element
                        element.appendChild(cameraElement);

                        // Optional: Add error handling for the image
                        cameraElement.onerror = () => {
                            console.error(`Camera Plugin: Error loading camera feed from ${domainObject.cameraFeedUrl}.`);
                            // You could display an error message on the screen here
                            if (cameraElement && cameraElement.parentElement) {
                                cameraElement.parentElement.removeChild(cameraElement);
                                const errorElement = document.createElement('div');
                                errorElement.style.textAlign = 'center';
                                errorElement.style.marginTop = '20px';
                                errorElement.style.color = 'red';
                                errorElement.textContent = 'Error loading camera feed. Check URL and network.';
                                element.appendChild(errorElement);
                            }
                        };
                    },
                    onEditModeChange(editMode) {
                        // Optional: Handle changes to edit mode if needed
                    },
                    destroy: function () {
                        // Clean up the image element when the view is destroyed
                        if (cameraElement && cameraElement.parentElement) {
                            cameraElement.parentElement.removeChild(cameraElement);
                        }
                        cameraElement = null;
                        console.log('Camera View destroyed.');
                    }
                };
            }
        });
        // --- End Define view provider ---

        return {
            destroy: () => { }
        };
    };
}
