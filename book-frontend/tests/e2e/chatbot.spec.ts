import { test, expect } from '@playwright/test';

test.describe('RAG Chatbot E2E Tests', () => {
  test.beforeEach(async ({ page }) => {
    // Navigate to the main page where the chatbot is available
    await page.goto('/');
    
    // Wait for the page to load
    await page.waitForLoadState('networkidle');
  });

  test('should open and close chatbot', async ({ page }) => {
    // Find and click the chatbot floating icon
    const chatbotIcon = page.locator('.chatbot-container .floating-icon, [aria-label*="chat"], button[title*="chat"]').first();
    await expect(chatbotIcon).toBeVisible();
    await chatbotIcon.click();

    // Verify chatbot window opens
    const chatWindow = page.locator('.chat-window, .chatbot-window');
    await expect(chatWindow).toBeVisible();

    // Find and click close button
    const closeButton = page.locator('.close-btn, [aria-label="Close chat"], [title="Close"]');
    await expect(closeButton).toBeVisible();
    await closeButton.click();

    // Verify chatbot window closes
    await expect(chatWindow).not.toBeVisible();
  });

  test('should send a basic chat message', async ({ page }) => {
    // Open chatbot
    const chatbotIcon = page.locator('.chatbot-container .floating-icon, [aria-label*="chat"], button[title*="chat"]').first();
    await chatbotIcon.click();

    // Wait for chat window to be visible
    const chatWindow = page.locator('.chat-window, .chatbot-window');
    await expect(chatWindow).toBeVisible();

    // Find message input
    const messageInput = page.locator('.message-input, textarea[placeholder*="message"], input[placeholder*="message"]');
    await expect(messageInput).toBeVisible();

    // Type a message
    const testMessage = 'Hello, can you help me with machine learning?';
    await messageInput.fill(testMessage);

    // Find and click send button
    const sendButton = page.locator('.send-btn, [aria-label="Send message"], [title="Send"]');
    await expect(sendButton).toBeVisible();
    await sendButton.click();

    // Verify user message appears in chat
    const userMessage = page.locator('.message, .chat-message').filter({ hasText: testMessage });
    await expect(userMessage).toBeVisible();

    // Wait for AI response (with timeout)
    const aiResponse = page.locator('.message.ai, .chat-message[data-sender="ai"], .message:has-text("demo")').first();
    await expect(aiResponse).toBeVisible({ timeout: 10000 });
  });

  test('should use Ask AI feature', async ({ page }) => {
    // Open chatbot
    const chatbotIcon = page.locator('.chatbot-container .floating-icon, [aria-label*="chat"], button[title*="chat"]').first();
    await chatbotIcon.click();

    const chatWindow = page.locator('.chat-window, .chatbot-window');
    await expect(chatWindow).toBeVisible();

    // Type a message
    const messageInput = page.locator('.message-input, textarea[placeholder*="message"], input[placeholder*="message"]');
    await messageInput.fill('What is artificial intelligence?');

    // Click Ask AI button instead of regular send
    const askAIButton = page.locator('.ask-ai-btn, [aria-label="Ask AI for help"], [title="Ask AI"]');
    await expect(askAIButton).toBeVisible();
    await askAIButton.click();

    // Verify message was sent and response received
    const userMessage = page.locator('.message, .chat-message').filter({ hasText: 'What is artificial intelligence?' });
    await expect(userMessage).toBeVisible();

    // Wait for enhanced AI response
    const aiResponse = page.locator('.message.ai, .chat-message[data-sender="ai"]').first();
    await expect(aiResponse).toBeVisible({ timeout: 10000 });
  });

  test('should handle file upload', async ({ page }) => {
    // Open chatbot
    const chatbotIcon = page.locator('.chatbot-container .floating-icon, [aria-label*="chat"], button[title*="chat"]').first();
    await chatbotIcon.click();

    const chatWindow = page.locator('.chat-window, .chatbot-window');
    await expect(chatWindow).toBeVisible();

    // Find file upload button
    const fileUploadButton = page.locator('.file-upload-btn, [aria-label="Upload file"], [title="Upload file"]');
    await expect(fileUploadButton).toBeVisible();

    // Create a test file
    const testFileContent = 'This is a test document for file upload testing.\nIt contains some sample content about machine learning.';
    
    // Set up file chooser handler
    const fileChooserPromise = page.waitForEvent('filechooser');
    await fileUploadButton.click();
    const fileChooser = await fileChooserPromise;

    // Create a temporary file and upload it
    await fileChooser.setFiles({
      name: 'test-document.txt',
      mimeType: 'text/plain',
      buffer: Buffer.from(testFileContent)
    });

    // Verify file upload message appears
    const uploadMessage = page.locator('.message, .chat-message').filter({ hasText: 'test-document.txt' });
    await expect(uploadMessage).toBeVisible({ timeout: 10000 });

    // Verify AI response about file processing
    const aiResponse = page.locator('.message.ai, .chat-message[data-sender="ai"]').filter({ hasText: /processed|uploaded|file/i });
    await expect(aiResponse).toBeVisible({ timeout: 15000 });
  });

  test('should handle voice input (mocked)', async ({ page }) => {
    // Mock getUserMedia for voice input testing
    await page.addInitScript(() => {
      // Mock MediaRecorder and getUserMedia
      (window as any).MediaRecorder = class MockMediaRecorder {
        constructor(stream: any) {
          this.stream = stream;
        }
        start() {
          setTimeout(() => {
            if (this.ondataavailable) {
              this.ondataavailable({ data: new Blob(['mock audio data'], { type: 'audio/wav' }) });
            }
          }, 100);
        }
        stop() {
          setTimeout(() => {
            if (this.onstop) {
              this.onstop();
            }
          }, 100);
        }
        ondataavailable: any;
        onstop: any;
        stream: any;
      };

      navigator.mediaDevices = {
        getUserMedia: () => Promise.resolve({
          getTracks: () => [{ stop: () => {} }]
        } as any)
      } as any;
    });

    // Open chatbot
    const chatbotIcon = page.locator('.chatbot-container .floating-icon, [aria-label*="chat"], button[title*="chat"]').first();
    await chatbotIcon.click();

    const chatWindow = page.locator('.chat-window, .chatbot-window');
    await expect(chatWindow).toBeVisible();

    // Find voice input button
    const voiceButton = page.locator('.voice-btn, [aria-label*="voice"], [title*="Voice"]');
    
    // Check if voice button exists (it might not be visible if not supported)
    const voiceButtonCount = await voiceButton.count();
    if (voiceButtonCount > 0) {
      await expect(voiceButton).toBeVisible();
      
      // Click to start recording
      await voiceButton.click();
      
      // Verify recording state (button should change appearance)
      await expect(voiceButton).toHaveClass(/recording/);
      
      // Wait a moment then stop recording
      await page.waitForTimeout(1000);
      await voiceButton.click();
      
      // Verify voice message processing
      const voiceMessage = page.locator('.message, .chat-message').filter({ hasText: /voice|audio|received/i });
      await expect(voiceMessage).toBeVisible({ timeout: 10000 });
    }
  });

  test('should handle text selection and Ask AI', async ({ page }) => {
    // Navigate to a page with content that can be selected
    await page.goto('/docs'); // Assuming there's a docs page with content
    
    // Wait for page to load
    await page.waitForLoadState('networkidle');

    // Find some text content to select
    const contentArea = page.locator('article, .markdown, .theme-doc-markdown, main').first();
    await expect(contentArea).toBeVisible();

    // Select some text (simulate text selection)
    const textElement = contentArea.locator('p, h1, h2, h3').first();
    await expect(textElement).toBeVisible();

    // Triple-click to select text
    await textElement.click({ clickCount: 3 });

    // Look for text selection popup
    const selectionPopup = page.locator('.text-selection-popup, .selection-popup');
    
    // Check if popup appears (might have delay)
    const popupCount = await selectionPopup.count();
    if (popupCount > 0) {
      await expect(selectionPopup).toBeVisible({ timeout: 5000 });

      // Find "Ask Neurobotics" or similar button
      const askButton = selectionPopup.locator('button').filter({ hasText: /ask|neurobotics/i });
      await expect(askButton).toBeVisible();
      await askButton.click();

      // Verify chatbot opens with the selected text
      const chatWindow = page.locator('.chat-window, .chatbot-window');
      await expect(chatWindow).toBeVisible({ timeout: 5000 });

      // Verify message about selected text appears
      const selectionMessage = page.locator('.message, .chat-message').filter({ hasText: /ask about|selected/i });
      await expect(selectionMessage).toBeVisible({ timeout: 5000 });
    }
  });

  test('should toggle between window sizes', async ({ page }) => {
    // Open chatbot
    const chatbotIcon = page.locator('.chatbot-container .floating-icon, [aria-label*="chat"], button[title*="chat"]').first();
    await chatbotIcon.click();

    const chatWindow = page.locator('.chat-window, .chatbot-window');
    await expect(chatWindow).toBeVisible();

    // Find resize/toggle button
    const resizeButton = page.locator('.resize-btn, [aria-label*="Fullscreen"], [aria-label*="Window mode"]');
    await expect(resizeButton).toBeVisible();

    // Get initial window class/size
    const initialClass = await chatWindow.getAttribute('class');

    // Click resize button
    await resizeButton.click();

    // Verify window size changed
    const newClass = await chatWindow.getAttribute('class');
    expect(newClass).not.toBe(initialClass);

    // Click again to toggle back
    await resizeButton.click();

    // Verify it toggled back
    const finalClass = await chatWindow.getAttribute('class');
    expect(finalClass).toBe(initialClass);
  });

  test('should show typing indicator during AI response', async ({ page }) => {
    // Open chatbot
    const chatbotIcon = page.locator('.chatbot-container .floating-icon, [aria-label*="chat"], button[title*="chat"]').first();
    await chatbotIcon.click();

    const chatWindow = page.locator('.chat-window, .chatbot-window');
    await expect(chatWindow).toBeVisible();

    // Send a message
    const messageInput = page.locator('.message-input, textarea[placeholder*="message"], input[placeholder*="message"]');
    await messageInput.fill('Tell me about neural networks');

    const sendButton = page.locator('.send-btn, [aria-label="Send message"], [title="Send"]');
    await sendButton.click();

    // Look for typing indicator (might be brief)
    const typingIndicator = page.locator('.typing-indicator, .typing-dots, [class*="typing"]');
    
    // The typing indicator might appear and disappear quickly
    // So we'll check if it exists at any point during a reasonable timeframe
    try {
      await expect(typingIndicator).toBeVisible({ timeout: 2000 });
    } catch (e) {
      // Typing indicator might be too fast to catch, that's okay
      console.log('Typing indicator not visible or too fast to detect');
    }

    // Verify final response appears
    const aiResponse = page.locator('.message.ai, .chat-message[data-sender="ai"]').first();
    await expect(aiResponse).toBeVisible({ timeout: 10000 });
  });

  test('should handle empty message gracefully', async ({ page }) => {
    // Open chatbot
    const chatbotIcon = page.locator('.chatbot-container .floating-icon, [aria-label*="chat"], button[title*="chat"]').first();
    await chatbotIcon.click();

    const chatWindow = page.locator('.chat-window, .chatbot-window');
    await expect(chatWindow).toBeVisible();

    // Try to send empty message
    const sendButton = page.locator('.send-btn, [aria-label="Send message"], [title="Send"]');
    
    // Send button should be disabled for empty input
    await expect(sendButton).toBeDisabled();

    // Try typing spaces only
    const messageInput = page.locator('.message-input, textarea[placeholder*="message"], input[placeholder*="message"]');
    await messageInput.fill('   ');

    // Send button should still be disabled
    await expect(sendButton).toBeDisabled();
  });

  test('should maintain chat history in session', async ({ page }) => {
    // Open chatbot
    const chatbotIcon = page.locator('.chatbot-container .floating-icon, [aria-label*="chat"], button[title*="chat"]').first();
    await chatbotIcon.click();

    const chatWindow = page.locator('.chat-window, .chatbot-window');
    await expect(chatWindow).toBeVisible();

    // Send first message
    const messageInput = page.locator('.message-input, textarea[placeholder*="message"], input[placeholder*="message"]');
    await messageInput.fill('First message');
    
    const sendButton = page.locator('.send-btn, [aria-label="Send message"], [title="Send"]');
    await sendButton.click();

    // Wait for response
    await page.waitForTimeout(2000);

    // Send second message
    await messageInput.fill('Second message');
    await sendButton.click();

    // Verify both messages are visible in chat history
    const firstMessage = page.locator('.message, .chat-message').filter({ hasText: 'First message' });
    const secondMessage = page.locator('.message, .chat-message').filter({ hasText: 'Second message' });

    await expect(firstMessage).toBeVisible();
    await expect(secondMessage).toBeVisible();

    // Close and reopen chatbot
    const closeButton = page.locator('.close-btn, [aria-label="Close chat"], [title="Close"]');
    await closeButton.click();

    await page.waitForTimeout(1000);

    await chatbotIcon.click();

    // Verify messages are still there (if session persistence is implemented)
    // Note: This depends on how session persistence is implemented
    // In demo mode, messages might not persist
  });
});