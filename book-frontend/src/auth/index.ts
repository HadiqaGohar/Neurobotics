/**
 * Authentication module exports
 */

// Main components
export { default as AuthContext, AuthProvider, useAuth, withAuth } from './AuthContext';
export { default as LoginForm } from './LoginForm';
export { default as SignupForm } from './SignupForm';\nexport { default as UserProfile } from './UserProfile';
export { 
  default as ProtectedRoute, 
  withProtectedRoute, 
  useRequireAuth,
  PermissionRoute,
  RoleRoute 
} from './ProtectedRoute';
export { default as SessionWarning } from './SessionWarning';

// API and utilities
export { authAPI, authUtils } from './authAPI';
export { default as SessionManager, useSessionManager } from './sessionUtils';
export { default as AuthStorage } from './authStorage';
export { default as AuthMiddleware, useAuthMiddleware } from './authMiddleware';
export { default as RouteManager } from './routeConfig';

// Hooks
export {
  useAuthStatus,
  usePermissions,
  useSession,
  useAuthLoading,
  useUserProfile,
  useAuthRedirect,
  useTokenRefresh,
  useAuthEvents,
  useAuthDebug,
} from './authHooks';

// Types
export type {
  LoginRequest,
  SignupRequest,
  TokenResponse,
  User,
  UpdateProfileRequest,
} from './authAPI';

export type { SessionInfo } from './sessionUtils';
export type { AuthMiddlewareConfig } from './authMiddleware';
export type { RouteConfig, RouteGroup } from './routeConfig';