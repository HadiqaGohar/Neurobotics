/**
 * API client for authentication endpoints.
 */

import axios, { AxiosInstance, AxiosResponse } from 'axios';

// Types
interface LoginRequest {
  email: string;
  password: string;
}

interface SignupRequest {
  email: string;
  password: string;
  full_name?: string;
}

interface TokenResponse {
  access_token: string;
  refresh_token: string;
  token_type: string;
  user: User;
}

interface User {
  id: string;
  email: string;
  full_name?: string;
  is_active: boolean;
  created_at: string;
  last_login?: string;
}

interface UpdateProfileRequest {
  full_name?: string;
  current_password?: string;
  new_password?: string;
}

// API configuration
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api/v1';

class AuthAPI {
  private client: AxiosInstance;

  constructor() {
    this.client = axios.create({
      baseURL: `${API_BASE_URL}/auth`,
      headers: {
        'Content-Type': 'application/json',
      },
      timeout: 10000,
    });

    // Request interceptor to add auth token
    this.client.interceptors.request.use(
      (config) => {
        const token = localStorage.getItem('access_token');
        if (token) {
          config.headers.Authorization = `Bearer ${token}`;
        }
        return config;
      },
      (error) => {
        return Promise.reject(error);
      }
    );

    // Response interceptor to handle token refresh
    this.client.interceptors.response.use(
      (response) => response,
      async (error) => {
        const originalRequest = error.config;

        if (error.response?.status === 401 && !originalRequest._retry) {
          originalRequest._retry = true;

          try {
            const refreshToken = localStorage.getItem('refresh_token');
            if (refreshToken) {
              const response = await this.refreshToken(refreshToken);
              localStorage.setItem('access_token', response.access_token);
              localStorage.setItem('refresh_token', response.refresh_token);
              
              // Retry original request with new token
              originalRequest.headers.Authorization = `Bearer ${response.access_token}`;
              return this.client(originalRequest);
            }
          } catch (refreshError) {
            // Refresh failed, clear tokens and redirect to login
            localStorage.removeItem('access_token');
            localStorage.removeItem('refresh_token');
            window.location.href = '/login';
          }
        }

        return Promise.reject(error);
      }
    );
  }

  // Authentication methods
  async login(email: string, password: string): Promise<TokenResponse> {
    const response: AxiosResponse<TokenResponse> = await this.client.post('/login', {
      email,
      password,
    });
    return response.data;
  }

  async signup(email: string, password: string, fullName?: string): Promise<TokenResponse> {
    const response: AxiosResponse<TokenResponse> = await this.client.post('/register', {
      email,
      password,
      confirm_password: password, // Backend expects confirm_password
      full_name: fullName,
    });
    return response.data;
  }

  async logout(): Promise<void> {
    await this.client.post('/logout');
  }

  async refreshToken(refreshToken: string): Promise<TokenResponse> {
    // The backend expects refresh_token as a query parameter or form data
    const response: AxiosResponse<TokenResponse> = await this.client.post(`/refresh?refresh_token=${refreshToken}`);
    return response.data;
  }

  // User profile methods
  async getCurrentUser(): Promise<User> {
    const response: AxiosResponse<User> = await this.client.get('/me');
    return response.data;
  }

  async updateProfile(data: UpdateProfileRequest): Promise<User> {
    const response: AxiosResponse<User> = await this.client.put('/me', data);
    return response.data;
  }

  // OAuth methods (for future implementation)
  async initiateOAuth(provider: string): Promise<{ oauth_url: string; state: string }> {
    const response = await this.client.get(`/oauth/${provider}`);
    return response.data;
  }

  async handleOAuthCallback(provider: string, code: string, state: string): Promise<TokenResponse> {
    const response: AxiosResponse<TokenResponse> = await this.client.get(
      `/oauth/${provider}/callback?code=${code}&state=${state}`
    );
    return response.data;
  }

  // Password reset methods (for future implementation)
  async forgotPassword(email: string): Promise<{ message: string }> {
    const response = await this.client.post('/forgot-password', { email });
    return response.data;
  }

  async resetPassword(token: string, newPassword: string): Promise<{ message: string }> {
    const response = await this.client.post('/reset-password', {
      token,
      new_password: newPassword,
    });
    return response.data;
  }

  // Utility methods
  isTokenExpired(token: string): boolean {
    try {
      const payload = JSON.parse(atob(token.split('.')[1]));
      const currentTime = Date.now() / 1000;
      return payload.exp < currentTime;
    } catch (error) {
      return true;
    }
  }

  getTokenPayload(token: string): any {
    try {
      return JSON.parse(atob(token.split('.')[1]));
    } catch (error) {
      return null;
    }
  }

  // Check if user is authenticated
  isAuthenticated(): boolean {
    const token = localStorage.getItem('access_token');
    return token ? !this.isTokenExpired(token) : false;
  }

  // Get current user ID from token
  getCurrentUserId(): string | null {
    const token = localStorage.getItem('access_token');
    if (token) {
      const payload = this.getTokenPayload(token);
      return payload?.sub || null;
    }
    return null;
  }

  // Clear all authentication data
  clearAuth(): void {
    localStorage.removeItem('access_token');
    localStorage.removeItem('refresh_token');
  }
}

// Create and export singleton instance
export const authAPI = new AuthAPI();

// Export types for use in components
export type {
  LoginRequest,
  SignupRequest,
  TokenResponse,
  User,
  UpdateProfileRequest,
};

// Export utility functions
export const authUtils = {
  isTokenExpired: authAPI.isTokenExpired.bind(authAPI),
  getTokenPayload: authAPI.getTokenPayload.bind(authAPI),
  isAuthenticated: authAPI.isAuthenticated.bind(authAPI),
  getCurrentUserId: authAPI.getCurrentUserId.bind(authAPI),
  clearAuth: authAPI.clearAuth.bind(authAPI),
};