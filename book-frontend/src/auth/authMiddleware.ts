/**
 * Authentication middleware for API calls and route protection
 */

import { authAPI } from './authAPI';

export interface AuthMiddlewareConfig {
  requireAuth?: boolean;
  requiredPermissions?: string[];
  requiredRole?: string;
  onUnauthorized?: () => void;
  onForbidden?: () => void;
  autoRefresh?: boolean;
}

export class AuthMiddleware {
  private static config: AuthMiddlewareConfig = {
    requireAuth: true,
    autoRefresh: true,
  };

  /**
   * Configure global middleware settings
   */
  static configure(config: Partial<AuthMiddlewareConfig>): void {
    this.config = { ...this.config, ...config };
  }

  /**
   * Check if user is authenticated
   */
  static async checkAuthentication(): Promise<boolean> {
    try {
      const token = localStorage.getItem('access_token');
      if (!token) return false;

      // Check if token is expired
      if (authAPI.isTokenExpired(token)) {
        if (this.config.autoRefresh) {
          const refreshToken = localStorage.getItem('refresh_token');
          if (refreshToken) {
            try {
              const response = await authAPI.refreshToken(refreshToken);
              localStorage.setItem('access_token', response.access_token);
              localStorage.setItem('refresh_token', response.refresh_token);
              return true;
            } catch (error) {
              this.handleUnauthorized();
              return false;
            }
          }
        }
        return false;
      }

      return true;
    } catch (error) {
      return false;
    }
  }

  /**
   * Check user permissions
   */
  static async checkPermissions(permissions: string[]): Promise<boolean> {
    try {
      const user = await authAPI.getCurrentUser();
      // Basic permission check - in a real app, this would be more sophisticated
      return user.is_active; // Simplified for now
    } catch (error) {
      return false;
    }
  }

  /**
   * Check user role
   */
  static async checkRole(requiredRole: string): Promise<boolean> {
    try {
      const user = await authAPI.getCurrentUser();
      // Basic role check - in a real app, this would check actual roles
      return user.is_active && requiredRole === 'user';
    } catch (error) {
      return false;
    }
  }

  /**
   * Handle unauthorized access
   */
  static handleUnauthorized(): void {
    if (this.config.onUnauthorized) {
      this.config.onUnauthorized();
    } else {
      // Clear auth data and redirect to login
      localStorage.removeItem('access_token');
      localStorage.removeItem('refresh_token');
      window.location.href = '/auth/signin';
    }
  }

  /**
   * Handle forbidden access
   */
  static handleForbidden(): void {
    if (this.config.onForbidden) {
      this.config.onForbidden();
    } else {
      console.warn('Access forbidden - insufficient permissions');
    }
  }

  /**
   * Middleware function for API calls
   */
  static async apiMiddleware(
    apiCall: () => Promise<any>,
    config?: AuthMiddlewareConfig
  ): Promise<any> {
    const middlewareConfig = { ...this.config, ...config };

    // Check authentication if required
    if (middlewareConfig.requireAuth) {
      const isAuthenticated = await this.checkAuthentication();
      if (!isAuthenticated) {
        this.handleUnauthorized();
        throw new Error('Authentication required');
      }
    }

    // Check permissions if required
    if (middlewareConfig.requiredPermissions?.length) {
      const hasPermissions = await this.checkPermissions(middlewareConfig.requiredPermissions);
      if (!hasPermissions) {
        this.handleForbidden();
        throw new Error('Insufficient permissions');
      }
    }

    // Check role if required
    if (middlewareConfig.requiredRole) {
      const hasRole = await this.checkRole(middlewareConfig.requiredRole);
      if (!hasRole) {
        this.handleForbidden();
        throw new Error('Insufficient role');
      }
    }

    // Execute the API call
    try {
      return await apiCall();
    } catch (error: any) {
      // Handle specific HTTP errors
      if (error.response?.status === 401) {
        this.handleUnauthorized();
      } else if (error.response?.status === 403) {
        this.handleForbidden();
      }
      throw error;
    }
  }

  /**
   * Create a protected API function
   */
  static createProtectedAPI<T extends (...args: any[]) => Promise<any>>(
    apiFunction: T,
    config?: AuthMiddlewareConfig
  ): T {
    return (async (...args: Parameters<T>) => {
      return this.apiMiddleware(() => apiFunction(...args), config);
    }) as T;
  }

  /**
   * Route guard function
   */
  static async routeGuard(config?: AuthMiddlewareConfig): Promise<boolean> {
    const middlewareConfig = { ...this.config, ...config };

    try {
      // Check authentication
      if (middlewareConfig.requireAuth) {
        const isAuthenticated = await this.checkAuthentication();
        if (!isAuthenticated) {
          this.handleUnauthorized();
          return false;
        }
      }

      // Check permissions
      if (middlewareConfig.requiredPermissions?.length) {
        const hasPermissions = await this.checkPermissions(middlewareConfig.requiredPermissions);
        if (!hasPermissions) {
          this.handleForbidden();
          return false;
        }
      }

      // Check role
      if (middlewareConfig.requiredRole) {
        const hasRole = await this.checkRole(middlewareConfig.requiredRole);
        if (!hasRole) {
          this.handleForbidden();
          return false;
        }
      }

      return true;
    } catch (error) {
      console.error('Route guard error:', error);
      return false;
    }
  }

  /**
   * Get authorization headers for API calls
   */
  static getAuthHeaders(): { [key: string]: string } {
    const token = localStorage.getItem('access_token');
    return token ? { Authorization: `Bearer ${token}` } : {};
  }

  /**
   * Create an authenticated fetch wrapper
   */
  static async authenticatedFetch(
    url: string,
    options: RequestInit = {},
    config?: AuthMiddlewareConfig
  ): Promise<Response> {
    return this.apiMiddleware(async () => {
      const authHeaders = this.getAuthHeaders();
      const headers = {
        'Content-Type': 'application/json',
        ...authHeaders,
        ...options.headers,
      };

      const response = await fetch(url, {
        ...options,
        headers,
      });

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      return response;
    }, config);
  }
}

/**
 * React hook for using auth middleware
 */
export const useAuthMiddleware = () => {
  const checkAuth = async (config?: AuthMiddlewareConfig) => {
    return AuthMiddleware.routeGuard(config);
  };

  const protectedFetch = async (
    url: string,
    options?: RequestInit,
    config?: AuthMiddlewareConfig
  ) => {
    return AuthMiddleware.authenticatedFetch(url, options, config);
  };

  const createProtectedAPI = <T extends (...args: any[]) => Promise<any>>(
    apiFunction: T,
    config?: AuthMiddlewareConfig
  ) => {
    return AuthMiddleware.createProtectedAPI(apiFunction, config);
  };

  return {
    checkAuth,
    protectedFetch,
    createProtectedAPI,
    getAuthHeaders: AuthMiddleware.getAuthHeaders,
  };
};

export default AuthMiddleware;