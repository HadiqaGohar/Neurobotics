import { useState, useEffect } from 'react';
import { useAuth } from './AuthContext';

interface AuthSnapshot {
  user: any;
  isAuthenticated: boolean;
  deviceId: string;
  timestamp: number;
}

interface ValidationResult {
  isValid: boolean;
  issues: string[];
  recommendations: string[];
}

export const useAuthStateManager = () => {
  const { user, isAuthenticated } = useAuth();
  const [snapshot, setSnapshot] = useState<AuthSnapshot | null>(null);

  useEffect(() => {
    if (isAuthenticated && user) {
      const newSnapshot: AuthSnapshot = {
        user,
        isAuthenticated,
        deviceId: getDeviceId(),
        timestamp: Date.now(),
      };
      setSnapshot(newSnapshot);
    }
  }, [user, isAuthenticated]);

  const getDeviceId = (): string => {
    let deviceId = localStorage.getItem('deviceId');
    if (!deviceId) {
      deviceId = 'device_' + Math.random().toString(36).substr(2, 9);
      localStorage.setItem('deviceId', deviceId);
    }
    return deviceId;
  };

  const exportState = (): string => {
    const exportData = {
      snapshot,
      preferences: localStorage.getItem('userPreferences'),
      timestamp: Date.now(),
    };
    return JSON.stringify(exportData, null, 2);
  };

  const importState = (data: string): boolean => {
    try {
      const parsed = JSON.parse(data);
      if (parsed.preferences) {
        localStorage.setItem('userPreferences', parsed.preferences);
      }
      return true;
    } catch (error) {
      console.error('Failed to import state:', error);
      return false;
    }
  };

  const validateState = (): ValidationResult => {
    const issues: string[] = [];
    const recommendations: string[] = [];

    if (!snapshot) {
      issues.push('No authentication snapshot available');
      recommendations.push('Please log in to create a valid session');
    }

    if (snapshot && !snapshot.user) {
      issues.push('User data is missing from snapshot');
      recommendations.push('Try logging out and logging back in');
    }

    if (snapshot && Date.now() - snapshot.timestamp > 24 * 60 * 60 * 1000) {
      issues.push('Authentication snapshot is older than 24 hours');
      recommendations.push('Consider refreshing your session');
    }

    return {
      isValid: issues.length === 0,
      issues,
      recommendations,
    };
  };

  return {
    snapshot,
    exportState,
    importState,
    validateState,
  };
};