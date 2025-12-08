/**
 * Session warning component to alert users before session expiration
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from './AuthContext';

interface SessionWarningProps {
  warningTimeMinutes?: number; // How many minutes before expiration to show warning
  sessionTimeoutMinutes?: number; // Total session timeout in minutes
}

const SessionWarning: React.FC<SessionWarningProps> = ({
  warningTimeMinutes = 5,
  sessionTimeoutMinutes = 30,
}) => {
  const { isAuthenticated, refreshToken, logout } = useAuth();
  const [showWarning, setShowWarning] = useState(false);
  const [timeLeft, setTimeLeft] = useState(0);

  useEffect(() => {
    if (!isAuthenticated) return;

    const checkSessionStatus = () => {
      const lastActivity = localStorage.getItem('lastActivity');
      if (!lastActivity) return;

      const lastActivityTime = parseInt(lastActivity);
      const currentTime = Date.now();
      const timeSinceActivity = currentTime - lastActivityTime;
      const sessionTimeoutMs = sessionTimeoutMinutes * 60 * 1000;
      const warningTimeMs = warningTimeMinutes * 60 * 1000;
      
      const timeUntilExpiration = sessionTimeoutMs - timeSinceActivity;
      
      if (timeUntilExpiration <= warningTimeMs && timeUntilExpiration > 0) {
        setShowWarning(true);
        setTimeLeft(Math.ceil(timeUntilExpiration / 1000));
      } else if (timeUntilExpiration <= 0) {
        // Session has expired
        logout();
      } else {
        setShowWarning(false);
      }
    };

    // Check session status every 10 seconds
    const interval = setInterval(checkSessionStatus, 10000);
    
    // Initial check
    checkSessionStatus();

    return () => clearInterval(interval);
  }, [isAuthenticated, warningTimeMinutes, sessionTimeoutMinutes, logout]);

  // Countdown timer
  useEffect(() => {
    if (!showWarning || timeLeft <= 0) return;

    const countdown = setInterval(() => {
      setTimeLeft(prev => {
        if (prev <= 1) {
          setShowWarning(false);
          logout();
          return 0;
        }
        return prev - 1;
      });
    }, 1000);

    return () => clearInterval(countdown);
  }, [showWarning, timeLeft, logout]);

  const handleExtendSession = async () => {
    try {
      // Update activity timestamp
      localStorage.setItem('lastActivity', Date.now().toString());
      
      // Optionally refresh the token
      await refreshToken();
      
      setShowWarning(false);
    } catch (error) {
      console.error('Failed to extend session:', error);
      logout();
    }
  };

  const handleLogout = () => {
    logout();
  };

  if (!showWarning) return null;

  const minutes = Math.floor(timeLeft / 60);
  const seconds = timeLeft % 60;

  return (
    <div className="session-warning-overlay">
      <div className="session-warning-modal">
        <div className="session-warning-header">
          <h3>⏰ Session Expiring Soon</h3>
        </div>
        
        <div className="session-warning-content">
          <p>
            Your session will expire in{' '}
            <strong>
              {minutes}:{seconds.toString().padStart(2, '0')}
            </strong>
          </p>
          <p>Would you like to extend your session?</p>
        </div>
        
        <div className="session-warning-actions">
          <button
            onClick={handleExtendSession}
            className="session-warning-button extend"
          >
            Extend Session
          </button>
          <button
            onClick={handleLogout}
            className="session-warning-button logout"
          >
            Logout
          </button>
        </div>
      </div>
    </div>
  );
};

export default SessionWarning;