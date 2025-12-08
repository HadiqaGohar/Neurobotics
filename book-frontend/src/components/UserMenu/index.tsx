/**
 * User menu component for authenticated users
 */

import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '../../auth/AuthContext';
import './styles.css';

interface UserMenuProps {
  className?: string;
}

const UserMenu: React.FC<UserMenuProps> = ({ className = '' }) => {
  const { user, isAuthenticated, logout } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }

    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [isOpen]);

  // Close menu on escape key
  useEffect(() => {
    const handleEscape = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener('keydown', handleEscape);
    }

    return () => {
      document.removeEventListener('keydown', handleEscape);
    };
  }, [isOpen]);

  const handleLogout = async () => {
    try {
      await logout();
      setIsOpen(false);
    } catch (error) {
      console.error('Logout failed:', error);
    }
  };

  const toggleMenu = () => {
    setIsOpen(!isOpen);
  };

  const handleMenuItemClick = () => {
    setIsOpen(false);
  };

  if (!isAuthenticated || !user) {
    return (
      <div className={`user-menu-container ${className}`}>
        <div className="auth-buttons">
          <a href="/auth/signin" className="auth-button signin">
            Sign In
          </a>
          <a href="/auth/signup" className="auth-button signup">
            Sign Up
          </a>
        </div>
      </div>
    );
  }

  const getInitials = (name: string): string => {
    return name
      .split(' ')
      .map(word => word.charAt(0).toUpperCase())
      .join('')
      .substring(0, 2);
  };

  const displayName = user.full_name || user.email.split('@')[0];
  const initials = getInitials(displayName);

  return (
    <div className={`user-menu-container ${className}`} ref={menuRef}>
      <button
        className={`user-menu-trigger ${isOpen ? 'active' : ''}`}
        onClick={toggleMenu}
        aria-expanded={isOpen}
        aria-haspopup="true"
        aria-label="User menu"
      >
        <div className="user-avatar">
          <span className="user-initials">{initials}</span>
        </div>
        <div className="user-info">
          <span className="user-name">{displayName}</span>
          <span className="user-email">{user.email}</span>
        </div>
        <svg
          className={`chevron ${isOpen ? 'rotated' : ''}`}
          width="16"
          height="16"
          viewBox="0 0 16 16"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path
            d="M4 6L8 10L12 6"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        </svg>
      </button>

      {isOpen && (
        <div className="user-menu-dropdown">


          <div className="menu-divider"></div>

          <div className="menu-items">
            <a
              href="/auth/profile"
              className="menu-item"
              onClick={handleMenuItemClick}
            >
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                <path
                  d="M8 8C10.2091 8 12 6.20914 12 4C12 1.79086 10.2091 0 8 0C5.79086 0 4 1.79086 4 4C4 6.20914 5.79086 8 8 8Z"
                  fill="currentColor"
                />
                <path
                  d="M8 10C3.58172 10 0 13.5817 0 18H16C16 13.5817 12.4183 10 8 10Z"
                  fill="currentColor"
                />
              </svg>
              <span>Profile</span>
            </a>

            <a
              href="/personalization"
              className="menu-item"
              onClick={handleMenuItemClick}
            >
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                <path
                  d="M8 0C8.55228 0 9 0.447715 9 1V2.05493C10.9463 2.27677 12.7232 3.05365 14.1421 4.47251L14.8492 3.76541C15.2397 3.37488 15.8729 3.37488 16.2634 3.76541C16.6539 4.15593 16.6539 4.78909 16.2634 5.17962L15.5563 5.88672C16.9752 7.30558 17.752 9.08246 17.9739 11.0288H19C19.5523 11.0288 20 11.4765 20 12.0288C20 12.5811 19.5523 13.0288 19 13.0288H17.9739C17.752 14.9751 16.9752 16.752 15.5563 18.1709L16.2634 18.878C16.6539 19.2685 16.6539 19.9017 16.2634 20.2922C15.8729 20.6827 15.2397 20.6827 14.8492 20.2922L14.1421 19.5851C12.7232 21.004 10.9463 21.7808 9 22.0027V23C9 23.5523 8.55228 24 8 24C7.44772 24 7 23.5523 7 23V22.0027C5.05365 21.7808 3.27677 21.004 1.85791 19.5851L1.15081 20.2922C0.760284 20.6827 0.127118 20.6827 -0.263406 20.2922C-0.65393 19.9017 -0.65393 19.2685 -0.263406 18.878L0.443695 18.1709C-0.975163 16.752 -1.75204 14.9751 -1.97388 13.0288H-3C-3.55228 13.0288 -4 12.5811 -4 12.0288C-4 11.4765 -3.55228 11.0288 -3 11.0288H-1.97388C-1.75204 9.08246 -0.975163 7.30558 0.443695 5.88672L-0.263406 5.17962C-0.65393 4.78909 -0.65393 4.15593 -0.263406 3.76541C0.127118 3.37488 0.760284 3.37488 1.15081 3.76541L1.85791 4.47251C3.27677 3.05365 5.05365 2.27677 7 2.05493V1C7 0.447715 7.44772 0 8 0Z"
                  fill="currentColor"
                />
                <circle cx="8" cy="12" r="4" fill="white" />
              </svg>
              <span>Preferences</span>
            </a>

            <div className="menu-item disabled">
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                <path
                  d="M2 4C2 2.89543 2.89543 2 4 2H12C13.1046 2 14 2.89543 14 4V12C14 13.1046 13.1046 14 12 14H4C2.89543 14 2 13.1046 2 12V4Z"
                  stroke="currentColor"
                  strokeWidth="2"
                  fill="none"
                />
                <path
                  d="M6 8L8 10L10 6"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              </svg>
              <span>Dashboard</span>
              <span className="coming-soon">Soon</span>
            </div>
          </div>

          <div className="menu-divider"></div>

          <div className="menu-items">
            <button
              className="menu-item logout"
              onClick={handleLogout}
            >
              <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
                <path
                  d="M6 2H4C3.44772 2 3 2.44772 3 3V13C3 13.5523 3.44772 14 4 14H6"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
                <path
                  d="M11 6L13 8L11 10"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
                <path
                  d="M13 8H7"
                  stroke="currentColor"
                  strokeWidth="2"
                  strokeLinecap="round"
                  strokeLinejoin="round"
                />
              </svg>
              <span>Sign Out</span>
            </button>
          </div>

          <div className="menu-footer">
            <div className="account-info">
              <div className="info-item">
                <span className="label">Member since</span>
                <span className="value">{new Date(user.created_at).toLocaleDateString()}</span>
              </div>
              {user.last_login && (
                <div className="info-item">
                  <span className="label">Last login</span>
                  <span className="value">{new Date(user.last_login).toLocaleDateString()}</span>
                </div>
              )}
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default UserMenu;