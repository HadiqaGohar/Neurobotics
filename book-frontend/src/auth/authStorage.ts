/**
 * Authentication storage utilities for persistent state management
 */

interface AuthStorageData {
  access_token: string | null;
  refresh_token: string | null;
  user: any | null;
  lastActivity: number;
  sessionStart: number;
  rememberMe: boolean;
  rememberedEmail: string | null;
}

export class AuthStorage {
  private static readonly STORAGE_KEY = 'auth_state';
  private static readonly SECURE_STORAGE_KEY = 'auth_secure';

  /**
   * Save authentication state to storage
   */
  static saveAuthState(data: Partial<AuthStorageData>): void {
    try {
      const currentData = this.getAuthState();
      const updatedData = { ...currentData, ...data };
      
      // Store non-sensitive data in localStorage
      const publicData = {
        user: updatedData.user,
        lastActivity: updatedData.lastActivity,
        sessionStart: updatedData.sessionStart,
        rememberMe: updatedData.rememberMe,
        rememberedEmail: updatedData.rememberedEmail,
      };
      
      localStorage.setItem(this.STORAGE_KEY, JSON.stringify(publicData));
      
      // Store tokens separately (could be moved to secure storage in production)
      if (updatedData.access_token) {
        localStorage.setItem('access_token', updatedData.access_token);
      }
      if (updatedData.refresh_token) {
        localStorage.setItem('refresh_token', updatedData.refresh_token);
      }
    } catch (error) {
      console.error('Failed to save auth state:', error);
    }
  }

  /**
   * Get authentication state from storage
   */
  static getAuthState(): AuthStorageData {
    try {
      const publicDataStr = localStorage.getItem(this.STORAGE_KEY);
      const publicData = publicDataStr ? JSON.parse(publicDataStr) : {};
      
      return {
        access_token: localStorage.getItem('access_token'),
        refresh_token: localStorage.getItem('refresh_token'),
        user: publicData.user || null,
        lastActivity: publicData.lastActivity || Date.now(),
        sessionStart: publicData.sessionStart || Date.now(),
        rememberMe: publicData.rememberMe || false,
        rememberedEmail: publicData.rememberedEmail || null,
      };
    } catch (error) {
      console.error('Failed to get auth state:', error);
      return {
        access_token: null,
        refresh_token: null,
        user: null,
        lastActivity: Date.now(),
        sessionStart: Date.now(),
        rememberMe: false,
        rememberedEmail: null,
      };
    }
  }

  /**
   * Clear all authentication data from storage
   */
  static clearAuthState(): void {
    try {
      localStorage.removeItem(this.STORAGE_KEY);
      localStorage.removeItem('access_token');
      localStorage.removeItem('refresh_token');
      localStorage.removeItem('lastActivity');
      localStorage.removeItem('sessionStartTime');
      localStorage.removeItem('rememberMe');
      localStorage.removeItem('rememberedEmail');
    } catch (error) {
      console.error('Failed to clear auth state:', error);
    }
  }

  /**
   * Check if user should be remembered
   */
  static shouldRememberUser(): boolean {
    const data = this.getAuthState();
    return data.rememberMe && !!data.rememberedEmail;
  }

  /**
   * Get remembered email
   */
  static getRememberedEmail(): string | null {
    const data = this.getAuthState();
    return data.rememberMe ? data.rememberedEmail : null;
  }

  /**
   * Update user data in storage
   */
  static updateUser(user: any): void {
    this.saveAuthState({ user });
  }

  /**
   * Update tokens in storage
   */
  static updateTokens(access_token: string, refresh_token: string): void {
    this.saveAuthState({ access_token, refresh_token });
  }

  /**
   * Update remember me preference
   */
  static updateRememberMe(rememberMe: boolean, email?: string): void {
    this.saveAuthState({
      rememberMe,
      rememberedEmail: rememberMe ? email || null : null,
    });
  }

  /**
   * Check if storage is available
   */
  static isStorageAvailable(): boolean {
    try {
      const test = '__storage_test__';
      localStorage.setItem(test, test);
      localStorage.removeItem(test);
      return true;
    } catch (error) {
      return false;
    }
  }

  /**
   * Migrate old storage format to new format
   */
  static migrateStorage(): void {
    try {
      // Check if we have old format data
      const oldTokens = {
        access_token: localStorage.getItem('access_token'),
        refresh_token: localStorage.getItem('refresh_token'),
      };
      
      const oldRememberMe = localStorage.getItem('rememberMe') === 'true';
      const oldRememberedEmail = localStorage.getItem('rememberedEmail');
      
      // If we have old data but no new format, migrate
      const existingData = localStorage.getItem(this.STORAGE_KEY);
      if ((oldTokens.access_token || oldRememberMe) && !existingData) {
        this.saveAuthState({
          access_token: oldTokens.access_token,
          refresh_token: oldTokens.refresh_token,
          rememberMe: oldRememberMe,
          rememberedEmail: oldRememberedEmail,
          lastActivity: Date.now(),
          sessionStart: Date.now(),
        });
      }
    } catch (error) {
      console.error('Failed to migrate storage:', error);
    }
  }

  /**
   * Get storage size information
   */
  static getStorageInfo(): { used: number; available: number } {
    try {
      let used = 0;
      for (let key in localStorage) {
        if (localStorage.hasOwnProperty(key)) {
          used += localStorage[key].length + key.length;
        }
      }
      
      // Rough estimate of available space (5MB typical limit)
      const available = 5 * 1024 * 1024 - used;
      
      return { used, available };
    } catch (error) {
      return { used: 0, available: 0 };
    }
  }
}

export default AuthStorage;