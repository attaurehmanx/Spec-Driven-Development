/**
 * Text Selection Utility Functions
 * Handles getting selected text from the current page
 */

/**
 * Get currently selected text from the page
 * @returns {string} The selected text, or empty string if none selected
 */
export const getSelectedText = () => {
  // Standard browser API for getting selected text
  if (window.getSelection) {
    return window.getSelection().toString().trim();
  } else if (document.selection && document.selection.type !== 'Control') {
    // Fallback for older IE
    return document.selection.createRange().text.trim();
  }
  return '';
};

/**
 * Add event listener for text selection
 * @param {Function} callback - Function to call when text is selected
 */
export const addTextSelectionListener = (callback) => {
  const handleSelectionChange = () => {
    const selectedText = getSelectedText();
    if (selectedText) {
      callback(selectedText);
    }
  };

  document.addEventListener('mouseup', handleSelectionChange);
  document.addEventListener('keyup', handleSelectionChange);

  // Return a function to remove the listeners
  return () => {
    document.removeEventListener('mouseup', handleSelectionChange);
    document.removeEventListener('keyup', handleSelectionChange);
  };
};

/**
 * Check if text is currently selected on the page
 * @returns {boolean} True if text is selected, false otherwise
 */
export const hasSelectedText = () => {
  return getSelectedText().length > 0;
};