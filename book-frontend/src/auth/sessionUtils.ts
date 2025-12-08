/**
 * Session management utilities
 */

import React from 'react';

export interface SessionInfo {
  isActive: boolean;
  lastActivity: number;
  sessionStart: number;
  timeUntilExpiration: number;
  warningThreshold: number;
}

export class SessionManager {
  private static readonly SESSION_TIMEOUT = 30 * 60 * 1000; // 30 minutes
  private static readonly WARNING_TIME = 5 * 60 * 1000; // 5 minutes
  private static readonly ACTIVITY_KEY = 'lastActivity';
  private static readonly SESSION_START_KEY = 'sessionStartTime';

  /**
   * Initialize a new session
   */
  static startSession(): void {
    const now = Date.now();
    localStorage.setItem(this.SESSION_START_KEY, now.toString());
    this.updateActivity();
  }

  /**
   * Update the last activity timestamp
   */
  static updateActivity(): void {
    localStorage.setItem(this.ACTIVITY_KEY, Date.now().toString());
  }

  /**
   * Get current session information
   */
  static getSessionInfo(): SessionInfo {
    const lastActivity = this.getLastActivity();
    const sessionStart = this.getSessionStart();
    const currentTime = Date.now();
    
    const timeSinceActivity = currentTime - lastActivity;
    const timeUntilExpiration = this.SESSION_TIMEOUT - timeSinceActivity;
    
    return {
      isActive: timeUntilExpiration > 0,
      lastActivity,
      sessionStart,
      timeUntilExpiration: Math.max(0, timeUntilExpiration),
      warningThreshold: this.WARNING_TIME,
    };
  }

  /**
   * Check if session should show warning
   */
  static shouldShowWarning(): boolean {
    const sessionInfo = this.getSessionInfo();
    return sessionInfo.isActive && sessionInfo.timeUntilExpiration <= this.WARNING_TIME;
  }

  /**
   * Check if session has expired
   */
  static isSessionExpired(): boolean {
    const sessionInfo = this.getSessionInfo();
    return !sessionInfo.isActive;
  }

  /**
   * Get time remaining in session (in seconds)
   */
  static getTimeRemaining(): number {
    const sessionInfo = this.getSessionInfo();
    return Math.ceil(sessionInfo.timeUntilExpiration / 1000);
  }

  /**
   * Clear all session data
   */
  static clearSession(): void {
    localStorage.removeItem(this.ACTIVITY_KEY);
    localStorage.removeItem(this.SESSION_START_KEY);
  }

  /**
   * Get last activity timestamp
   */
  private static getLastActivity(): number {
    const stored = localStorage.getItem(this.ACTIVITY_KEY);
    return stored ? parseInt(stored) : Date.now();
  }

  /**
   * Get session start timestamp
   */
  private static getSessionStart(): number {
    const stored = localStorage.getItem(this.SESSION_START_KEY);
    return stored ? parseInt(stored) : Date.now();
  }

  /**
   * Format time remaining as MM:SS
   */
  static formatTimeRemaining(seconds: number): string {
    const minutes = Math.floor(seconds / 60);
    const remainingSeconds = seconds % 60;
    return `${minutes}:${remainingSeconds.toString().padStart(2, '0')}`;
  }

  /**
   * Set up activity listeners
   */
  static setupActivityListeners(): () => void {
    const events = ['mousedown', 'mousemove', 'keypress', 'scroll', 'touchstart', 'click'];
    
    const handleActivity = () => {
      this.updateActivity();
    };

    events.forEach(event => {
      document.addEventListener(event, handleActivity, true);
    });

    // Return cleanup function
    return () => {
      events.forEach(event => {
        document.removeEventListener(event, handleActivity, true);
      });
    };
  }

  /**
   * Create a session monitor that calls a callback when session status changes
   */
  static createSessionMonitor(
    onWarning: () => void,
    onExpired: () => void,
    checkInterval: number = 10000
  ): () => void {
    let wasWarning = false;
    let wasExpired = false;

    const checkSession = () => {
      const shouldWarn = this.shouldShowWarning();
      const isExpired = this.isSessionExpired();

      // Session expired
      if (isExpired && !wasExpired) {
        wasExpired = true;
        onExpired();
        return;
      }

      // Show warning
      if (shouldWarn && !wasWarning && !isExpired) {
        wasWarning = true;
        onWarning();
      }

      // Clear warning if session was extended
      if (!shouldWarn && wasWarning) {
        wasWarning = false;
      }
    };

    const interval = setInterval(checkSession, checkInterval);
    
    // Initial check
    checkSession();

    // Return cleanup function
    return () => {
      clearInterval(interval);
    };
  }
}

/**
 * React hook for session management
 */
export const useSessionManager = () => {
  const [sessionInfo, setSessionInfo] = React.useState<SessionInfo>(() => 
    SessionManager.getSessionInfo()
  );

  React.useEffect(() => {
    const updateSessionInfo = () => {
      setSessionInfo(SessionManager.getSessionInfo());
    };

    // Update session info every second
    const interval = setInterval(updateSessionInfo, 1000);

    // Set up activity listeners
    const cleanupActivity = SessionManager.setupActivityListeners();

    return () => {
      clearInterval(interval);
      cleanupActivity();
    };
  }, []);

  return {
    sessionInfo,
    updateActivity: SessionManager.updateActivity,
    clearSession: SessionManager.clearSession,
    formatTimeRemaining: SessionManager.formatTimeRemaining,
  };
};

export default SessionManager;