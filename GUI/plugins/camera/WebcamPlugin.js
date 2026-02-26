///plugins/camera/WebcamPlugin.js

const WEBCAM_FEED_KEY = 'webcam-feed';

window.WebcamPlugin = function WebcamPlugin() {
    return function install(openmct) {
        openmct.types.addType(WEBCAM_FEED_KEY, {
            name: 'Webcam Feed',
            description: 'Displays a live video feed from the local computer\'s webcam.',
            creatable: true,
            cssClass: 'icon-camera', // Using a camera icon
            initialize(domainObject) {
                // No specific properties needed for a basic webcam feed
            },
            form: [] // No configuration form for this simple plugin
        });

        openmct.objectViews.addProvider({
            key: 'webcam-feed-view',
            name: 'Webcam Feed View',
            canView: (domainObject) => {
                return domainObject.type === WEBCAM_FEED_KEY;
            },
            view: (domainObject) => {
                let videoElement = null;
                let mediaStream = null; // To hold the camera stream
                let statusMessageElement = null;
                let snapshotButton = null; // Reference to the new snapshot button (now the outer circle)
                let innerCircle = null; // Reintroduced: for the inner white circle

                const displayStatus = (message, type = 'info') => {
                    if (statusMessageElement) {
                        if (statusMessageElement.parentElement) {
                            statusMessageElement.parentElement.removeChild(statusMessageElement);
                        }
                        statusMessageElement = null;
                    }

                    statusMessageElement = document.createElement('div');
                    statusMessageElement.style.position = 'absolute';
                    statusMessageElement.style.top = '50%';
                    statusMessageElement.style.left = '50%';
                    statusMessageElement.style.transform = 'translate(-50%, -50%)';
                    statusMessageElement.style.padding = '10px 20px';
                    statusMessageElement.style.borderRadius = '5px';
                    statusMessageElement.style.zIndex = '10';
                    statusMessageElement.style.fontSize = '14px';
                    statusMessageElement.style.textAlign = 'center';

                    if (type === 'error') {
                        statusMessageElement.style.backgroundColor = 'rgba(217, 83, 79, 0.9)'; // Red
                        statusMessageElement.style.color = 'white';
                    } else if (type === 'warning') {
                        statusMessageElement.style.backgroundColor = 'rgba(240, 173, 78, 0.9)'; // Orange
                        statusMessageElement.style.color = 'black';
                    } else {
                        statusMessageElement.style.backgroundColor = 'rgba(92, 184, 92, 0.9)'; // Green
                        statusMessageElement.style.color = 'white';
                    }
                    statusMessageElement.textContent = message;

                    if (videoElement && videoElement.parentElement) {
                        videoElement.parentElement.appendChild(statusMessageElement);
                    }
                };

                const startWebcam = async () => {
                    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
                        displayStatus('Webcam not supported by this browser.', 'error');
                        console.error('Webcam not supported by this browser.');
                        return;
                    }

                    try {
                        displayStatus('Requesting webcam access...', 'info');
                        mediaStream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: 'user' } });
                        videoElement.srcObject = mediaStream;
                        videoElement.play();
                        videoElement.style.display = 'block';
                        if (statusMessageElement && statusMessageElement.parentElement) {
                            statusMessageElement.parentElement.removeChild(statusMessageElement);
                            statusMessageElement = null;
                        }
                        // Show the snapshot button once the stream starts
                        if (snapshotButton) {
                            snapshotButton.style.display = 'block'; // Make button visible
                        }
                        console.log('Webcam stream started.');
                    } catch (err) {
                        displayStatus('Failed to access webcam. Please ensure it\'s connected and permissions are granted.', 'error');
                        console.error('Error accessing webcam:', err);
                        videoElement.style.display = 'none';
                        // Hide the snapshot button on error
                        if (snapshotButton) {
                            snapshotButton.style.display = 'none';
                        }
                    }
                };

                const stopWebcam = () => {
                    if (mediaStream) {
                        mediaStream.getTracks().forEach(track => track.stop());
                        mediaStream = null;
                        videoElement.srcObject = null;
                        console.log('Webcam stream stopped.');
                    }
                    // Hide the snapshot button when stream stops
                    if (snapshotButton) {
                        snapshotButton.style.display = 'none';
                    }
                };

                // Function to take a snapshot from the video element
                const takeSnapshot = () => {
                    if (!videoElement || videoElement.paused || videoElement.ended || videoElement.readyState < videoElement.HAVE_CURRENT_DATA) {
                        console.warn('Cannot take snapshot: Video stream not ready.');
                        openmct.notifications.error('Snapshot failed: Video stream not ready.');
                        return;
                    }

                    const canvas = document.createElement('canvas');
                    canvas.width = videoElement.videoWidth;
                    canvas.height = videoElement.videoHeight;
                    const context = canvas.getContext('2d');
                    context.drawImage(videoElement, 0, 0, canvas.width, canvas.height);

                    const imageDataUrl = canvas.toDataURL('image/png');

                    // Create a temporary link element to trigger the download
                    const link = document.createElement('a');
                    link.href = imageDataUrl;
                    link.download = `webcam-snapshot-${Date.now()}.png`; // Suggested filename
                    document.body.appendChild(link);
                    link.click();
                    document.body.removeChild(link);
                    console.log('Snapshot captured and download initiated.');

                    openmct.notifications.info('Snapshot captured successfully!');
                };

                return {
                    show(element) {
                        // Create a container for video and button to manage layout
                        const container = document.createElement('div');
                        container.style.width = '100%';
                        container.style.height = '100%';
                        container.style.position = 'relative'; // For absolute positioning of status/button

                        // Create video element
                        videoElement = document.createElement('video');
                        videoElement.style.width = '100%';
                        videoElement.style.height = '100%'; // Video now takes full height
                        videoElement.style.objectFit = 'contain';
                        videoElement.style.backgroundColor = '#333';
                        videoElement.style.display = 'none';
                        videoElement.autoplay = true;
                        videoElement.playsInline = true;
                        container.appendChild(videoElement);

                        // Create snapshot button (now the outer white ring/border)
                        snapshotButton = document.createElement('button');
                        snapshotButton.style.position = 'absolute';
                        snapshotButton.style.bottom = '15px'; // Position it lower, relative to container bottom
                        snapshotButton.style.left = '50%';
                        snapshotButton.style.transform = 'translateX(-50%)';
                        snapshotButton.style.width = '60px'; // Size for the outer circle
                        snapshotButton.style.height = '60px'; // Size for the outer circle
                        snapshotButton.style.backgroundColor = 'transparent'; // Transparent background for the outer ring
                        snapshotButton.style.border = '2px solid white'; // Thin white border for the outer ring
                        snapshotButton.style.borderRadius = '50%'; // Make it circular
                        snapshotButton.style.cursor = 'pointer';
                        snapshotButton.style.display = 'none'; // Hidden until stream starts
                        snapshotButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)'; // Add shadow
                        snapshotButton.style.outline = 'none'; // Remove outline on focus
                        
                        // Add active state for click effect (outer circle press)
                        snapshotButton.addEventListener('mousedown', () => {
                            snapshotButton.style.transform = 'translateX(-50%) scale(0.95)';
                            snapshotButton.style.boxShadow = '0 2px 4px rgba(0,0,0,0.2)';
                        });
                        snapshotButton.addEventListener('mouseup', () => {
                            snapshotButton.style.transform = 'translateX(-50%) scale(1)';
                            snapshotButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
                        });
                        snapshotButton.addEventListener('mouseleave', () => {
                             // Reset if mouse leaves while held down
                            snapshotButton.style.transform = 'translateX(-50%) scale(1)';
                            snapshotButton.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
                        });

                        snapshotButton.addEventListener('click', takeSnapshot);
                        container.appendChild(snapshotButton);

                        // Create inner white circle (this is the actual capture indicator)
                        innerCircle = document.createElement('div');
                        innerCircle.style.width = '45px'; // Smaller than outer circle, adjusted for proportion
                        innerCircle.style.height = '45px'; // Smaller than outer circle
                        innerCircle.style.backgroundColor = 'white'; // Solid white background for inner circle
                        innerCircle.style.border = 'none'; // No border on the inner solid white circle
                        innerCircle.style.borderRadius = '50%'; // Make it circular
                        innerCircle.style.position = 'absolute'; // Position relative to its parent (snapshotButton)
                        innerCircle.style.top = '50%';
                        innerCircle.style.left = '50%';
                        innerCircle.style.transform = 'translate(-50%, -50%)'; // Center it
                        innerCircle.style.boxSizing = 'border-box'; // Include padding and border in element's total width and height
                        snapshotButton.appendChild(innerCircle); // Append inner circle to the button

                        element.appendChild(container); // Append the main container to the Open MCT element

                        // Start the webcam when the view is shown
                        startWebcam();
                    },
                    onEditModeChange(editMode) {
                        // Stop webcam when entering edit mode, restart when exiting
                        if (editMode) {
                            stopWebcam();
                            displayStatus('Webcam: In edit mode. Stream paused.', 'info');
                        } else {
                            startWebcam();
                        }
                    },
                    destroy: function () {
                        // Clean up event listener
                        if (snapshotButton) {
                            snapshotButton.removeEventListener('click', takeSnapshot);
                            snapshotButton.removeEventListener('mousedown', () => {});
                            snapshotButton.removeEventListener('mouseup', () => {});
                            snapshotButton.removeEventListener('mouseleave', () => {});

                            if (snapshotButton.parentElement) {
                                snapshotButton.parentElement.removeChild(snapshotButton);
                            }
                            snapshotButton = null;
                        }
                        // Inner circle cleanup
                        if (innerCircle && innerCircle.parentElement) {
                            innerCircle.parentElement.removeChild(innerCircle);
                            innerCircle = null;
                        }
                        stopWebcam();
                        if (videoElement && videoElement.parentElement) {
                            videoElement.parentElement.removeChild(videoElement);
                        }
                        videoElement = null;
                        if (statusMessageElement && statusMessageElement.parentElement) {
                            statusMessageElement.parentElement.removeChild(statusMessageElement);
                        }
                        statusMessageElement = null;
                        console.log('Webcam Feed View destroyed.');
                    }
                };
            }
        });

        return {
            destroy: () => {}
        };
    };
}
