import React from 'react';
import Layout from '@theme/Layout';
import styles from './chatbot-demo.module.css';

export default function ChatbotDemo(): JSX.Element {
  return (
    <Layout
      title="Chatbot Demo"
      description="Interactive Chatbot UI Demo Page">
      <div className={styles.container}>
        <div className={styles.hero}>
          <h1 className={styles.title}>🤖 Interactive Chatbot Demo</h1>
          <p className={styles.subtitle}>
            Experience our floating AI assistant with voice input, copy functionality, and smart help features
          </p>
        </div>

        <div className={styles.content}>
          <div className={styles.section}>
            <h2>✨ Features to Try</h2>
            <div className={styles.featureGrid}>
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>💬</div>
                <h3>Chat with AI</h3>
                <p>Click the floating icon in the bottom-right corner to start a conversation with our AI assistant.</p>
              </div>
              
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🎤</div>
                <h3>Voice Input</h3>
                <p>Use the microphone button to speak your messages instead of typing them.</p>
              </div>
              
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>📋</div>
                <h3>Copy Messages</h3>
                <p>Hover over any message and click the copy icon to save responses to your clipboard.</p>
              </div>
              
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>✨</div>
                <h3>Ask AI Help</h3>
                <p>Use the sparkle button for detailed explanations and enhanced AI assistance.</p>
              </div>
              
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>📱</div>
                <h3>Responsive Design</h3>
                <p>Resize your window or use mobile - the chatbot adapts to any screen size.</p>
              </div>
              
              <div className={styles.featureCard}>
                <div className={styles.featureIcon}>🔄</div>
                <h3>Resizable Window</h3>
                <p>Toggle between small and large chat window sizes using the resize button.</p>
              </div>
            </div>
          </div>

          <div className={styles.section}>
            <h2>🚀 Getting Started</h2>
            <ol className={styles.stepsList}>
              <li>Look for the floating chat icon in the bottom-right corner</li>
              <li>Click it to open the chat window</li>
              <li>Type a message or try "Hello, how can you help me?"</li>
              <li>Experiment with voice input and the Ask AI feature</li>
              <li>Try copying responses to your clipboard</li>
            </ol>
          </div>

          <div className={styles.section}>
            <h2>💡 Sample Questions to Try</h2>
            <div className={styles.sampleQuestions}>
              <div className={styles.questionCard}>
                <strong>General Chat:</strong>
                <p>"What can you help me with?"</p>
              </div>
              <div className={styles.questionCard}>
                <strong>Ask AI Help:</strong>
                <p>"Explain quantum computing" (use the ✨ button)</p>
              </div>
              <div className={styles.questionCard}>
                <strong>Voice Test:</strong>
                <p>Click the microphone and say "Hello there!"</p>
              </div>
              <div className={styles.questionCard}>
                <strong>Technical:</strong>
                <p>"How does machine learning work?"</p>
              </div>
            </div>
          </div>

          <div className={styles.callout}>
            <h3>🎯 Pro Tips</h3>
            <ul>
              <li>Use <kbd>Enter</kbd> to send messages quickly</li>
              <li>The chat remembers your conversation history</li>
              <li>Voice input requires microphone permissions</li>
              <li>All messages can be copied with one click</li>
              <li>The AI provides more detailed help when using the ✨ Ask AI button</li>
            </ul>
          </div>
        </div>
      </div>
    </Layout>
  );
}