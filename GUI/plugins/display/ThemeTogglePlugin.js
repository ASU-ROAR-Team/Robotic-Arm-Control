// plugins/display/ThemeToggleObjectPlugin.js


// Define the plugin factory function and make it globally accessible
window.ThemeToggleObjectPlugin = function ThemeToggleObjectPlugin() {
    // Define the key for your new object type
    const THEME_TOGGLE_KEY = 'themeToggle';

    return function install(openmct) {
        // 1. Define a new object type for the theme toggle
        openmct.types.addType(THEME_TOGGLE_KEY, {
            // Removed emoji from name, relying on cssClass for the icon
            name: 'Theme Switch',
            description: 'A toggle switch to change between Light (Snow) and Dark (Espresso) themes.',
            creatable: true,
            // CHANGED: Trying 'icon-gear' as a common alternative icon class
            cssClass: 'icon-brightness',
            initialize(domainObject) {
                domainObject.name = domainObject.name || 'Theme Toggle';
            },
            form: [] // No specific properties to configure for this object
        });

        // 2. Define the view provider for the new object type
        openmct.objectViews.addProvider({
            key: 'theme-toggle-view',
            name: 'Theme Toggle View',
            canView: (domainObject) => {
                return domainObject.type === THEME_TOGGLE_KEY;
            },
            view: (domainObject) => {
                let toggleContainer = null;
                let toggleCheckbox = null;

                return {
                    show(element) {
                        // Get the current theme preference (default to snow for light mode)
                        const currentTheme = localStorage.getItem('openmct.theme') || 'snow';
                        const isLightMode = currentTheme === 'snow';

                        // Create a container for the toggle switch
                        toggleContainer = document.createElement('div');
                        toggleContainer.className = 'theme-toggle-container';
                        toggleContainer.style.display = 'flex';
                        toggleContainer.style.flexDirection = 'column';
                        toggleContainer.style.alignItems = 'center';
                        toggleContainer.style.justifyContent = 'center';
                        toggleContainer.style.height = '100%'; // Take full height of its parent
                        toggleContainer.style.width = '100%'; // Take full width of its parent

                        // Create the toggle switch HTML structure
                        toggleContainer.innerHTML = `
                            <style>
                                /* Basic styling for the toggle switch */
                                .switch-label {
                                    font-size: 0.9em;
                                    color: var(--color-body-fg); /* Use Open MCT's foreground color variable */
                                    margin-bottom: 5px;
                                    white-space: nowrap; /* Keep text on one line */
                                }

                                .toggle-switch {
                                    position: relative;
                                    display: inline-block;
                                    width: 60px; /* Width of the switch */
                                    height: 34px; /* Height of the switch */
                                }

                                .toggle-switch input {
                                    opacity: 0;
                                    width: 0;
                                    height: 0;
                                }

                                .slider {
                                    position: absolute;
                                    cursor: pointer;
                                    top: 0;
                                    left: 0;
                                    right: 0;
                                    bottom: 0;
                                    background-color: #ccc; /* Background for OFF state */
                                    transition: .4s;
                                    border-radius: 34px; /* Rounded corners for the track */
                                }

                                .slider:before {
                                    position: absolute;
                                    content: "";
                                    height: 26px; /* Height of the thumb */
                                    width: 26px; /* Width of the thumb */
                                    left: 4px; /* Initial position of the thumb */
                                    bottom: 4px; /* Initial position of the thumb */
                                    background-color: white; /* Color of the thumb */
                                    transition: .4s;
                                    border-radius: 50%; /* Make the thumb round */
                                }

                                input:checked + .slider {
                                    background-color: #2196F3; /* Background for ON state (blue) */
                                }

                                input:focus + .slider {
                                    box-shadow: 0 0 1px #2196F3;
                                }

                                input:checked + .slider:before {
                                    transform: translateX(26px); /* Move thumb to the right */
                                }

                                /* Optional: Add icons for light/dark mode */
                                .slider.round {
                                    display: flex;
                                    align-items: center;
                                    justify-content: space-between;
                                    padding: 0 5px;
                                }

                                .slider.round:after { /* Light mode icon */
                                    content: '☀️'; /* Sun emoji */
                                    font-size: 1.2em;
                                    position: absolute;
                                    right: 8px; /* Position to the right */
                                    color: white;
                                    opacity: 0;
                                    transition: opacity 0.4s ease-in-out;
                                }

                                .slider.round:before { /* Dark mode icon */
                                    content: '🌙'; /* Moon emoji */
                                    font-size: 1.2em;
                                    position: absolute;
                                    left: 8px; /* Position to the left */
                                    color: white;
                                    opacity: 0;
                                    transition: opacity 0.4s ease-in-out;
                                }

                                input:checked + .slider.round:after {
                                    opacity: 1; /* Show sun when checked (light mode) */
                                }

                                input:not(:checked) + .slider.round:before {
                                    opacity: 1; /* Show moon when unchecked (dark mode) */
                                }
                            </style>
                            <label class="switch-label">Dark Mode</label>
                            <label class="toggle-switch">
                                <input type="checkbox" id="themeToggleCheckbox">
                                <span class="slider round"></span>
                            </label>
                        `;

                        // Append the container to the provided element
                        element.appendChild(toggleContainer);

                        // Get reference to the checkbox
                        toggleCheckbox = toggleContainer.querySelector('#themeToggleCheckbox');
                        const labelElement = toggleContainer.querySelector('.switch-label');

                        // Set initial state of the checkbox and label
                        toggleCheckbox.checked = isLightMode;
                        labelElement.textContent = isLightMode ? 'Light Mode' : 'Dark Mode';

                        // Add event listener for the toggle switch
                        toggleCheckbox.addEventListener('change', () => {
                            const newTheme = toggleCheckbox.checked ? 'snow' : 'espresso';
                            localStorage.setItem('openmct.theme', newTheme);
                            console.log(`Theme preference updated to '${newTheme}'. Reloading application...`);
                            // Update the label text immediately before reload
                            labelElement.textContent = newTheme === 'snow' ? 'Light Mode' : 'Dark Mode';
                            window.location.reload(); // Reload the page to apply the new theme
                        });
                    },
                    destroy: function () {
                        // Clean up the elements when the view is destroyed
                        if (toggleContainer && toggleContainer.parentElement) {
                            toggleContainer.parentElement.removeChild(toggleContainer);
                        }
                        toggleContainer = null;
                        toggleCheckbox = null;
                        console.log('Theme Toggle View destroyed.');
                    }
                };
            }
        });

        // The install function can optionally return an object with a destroy method
        return {
            destroy: () => {
                // No specific cleanup needed for this simple plugin.
            }
        };
    };
}
