/**
 * Route configuration system for authentication and authorization
 */

export interface RouteConfig {
  path: string;
  requireAuth?: boolean;
  requiredPermissions?: string[];
  requiredRole?: string;
  redirectTo?: string;
  customValidator?: (user: any) => boolean;
  component?: React.ComponentType<any>;
  exact?: boolean;
}

export interface RouteGroup {
  name: string;
  basePath: string;
  defaultAuth?: boolean;
  defaultPermissions?: string[];
  defaultRole?: string;
  routes: RouteConfig[];
}

export class RouteManager {
  private static routes: Map<string, RouteConfig> = new Map();
  private static groups: Map<string, RouteGroup> = new Map();

  /**
   * Register a single route
   */
  static registerRoute(config: RouteConfig): void {
    this.routes.set(config.path, config);
  }

  /**
   * Register multiple routes
   */
  static registerRoutes(configs: RouteConfig[]): void {
    configs.forEach(config => this.registerRoute(config));
  }

  /**
   * Register a route group
   */
  static registerGroup(group: RouteGroup): void {
    this.groups.set(group.name, group);
    
    // Register individual routes with group defaults
    group.routes.forEach(route => {
      const fullPath = `${group.basePath}${route.path}`.replace(/\/+/g, '/');
      const routeConfig: RouteConfig = {
        ...route,
        path: fullPath,
        requireAuth: route.requireAuth ?? group.defaultAuth,
        requiredPermissions: route.requiredPermissions ?? group.defaultPermissions,
        requiredRole: route.requiredRole ?? group.defaultRole,
      };
      this.registerRoute(routeConfig);
    });
  }

  /**
   * Get route configuration
   */
  static getRoute(path: string): RouteConfig | undefined {
    return this.routes.get(path);
  }

  /**
   * Get all routes
   */
  static getAllRoutes(): RouteConfig[] {
    return Array.from(this.routes.values());
  }

  /**
   * Get routes by permission
   */
  static getRoutesByPermission(permission: string): RouteConfig[] {
    return this.getAllRoutes().filter(route => 
      route.requiredPermissions?.includes(permission)
    );
  }

  /**
   * Get routes by role
   */
  static getRoutesByRole(role: string): RouteConfig[] {
    return this.getAllRoutes().filter(route => 
      route.requiredRole === role
    );
  }

  /**
   * Check if user can access route
   */
  static canAccessRoute(
    path: string, 
    user: any, 
    checkPermission: (permission: string) => boolean,
    getUserRole: () => string | null
  ): boolean {
    const route = this.getRoute(path);
    if (!route) return true; // Allow access to unregistered routes

    // Check authentication requirement
    if (route.requireAuth && !user) {
      return false;
    }

    // Check role requirement
    if (route.requiredRole) {
      const userRole = getUserRole();
      if (userRole !== route.requiredRole) {
        return false;
      }
    }

    // Check permission requirements
    if (route.requiredPermissions?.length) {
      const hasAllPermissions = route.requiredPermissions.every(permission => 
        checkPermission(permission)
      );
      if (!hasAllPermissions) {
        return false;
      }
    }

    // Check custom validator
    if (route.customValidator && !route.customValidator(user)) {
      return false;
    }

    return true;
  }

  /**
   * Get accessible routes for user
   */
  static getAccessibleRoutes(
    user: any,
    checkPermission: (permission: string) => boolean,
    getUserRole: () => string | null
  ): RouteConfig[] {
    return this.getAllRoutes().filter(route => 
      this.canAccessRoute(route.path, user, checkPermission, getUserRole)
    );
  }

  /**
   * Clear all routes
   */
  static clearRoutes(): void {
    this.routes.clear();
    this.groups.clear();
  }

  /**
   * Get route statistics
   */
  static getRouteStats(): {
    totalRoutes: number;
    protectedRoutes: number;
    publicRoutes: number;
    permissionRoutes: number;
    roleRoutes: number;
  } {
    const routes = this.getAllRoutes();
    return {
      totalRoutes: routes.length,
      protectedRoutes: routes.filter(r => r.requireAuth).length,
      publicRoutes: routes.filter(r => !r.requireAuth).length,
      permissionRoutes: routes.filter(r => r.requiredPermissions?.length).length,
      roleRoutes: routes.filter(r => r.requiredRole).length,
    };
  }
}

// Predefined route configurations
export const defaultRoutes: RouteConfig[] = [
  {
    path: '/',
    requireAuth: false,
  },
  {
    path: '/auth/signin',
    requireAuth: false,
  },
  {
    path: '/auth/signup',
    requireAuth: false,
  },
  {
    path: '/dashboard',
    requireAuth: true,
    requiredPermissions: ['read'],
  },
  {
    path: '/profile',
    requireAuth: true,
    requiredPermissions: ['read'],
  },
  {
    path: '/settings',
    requireAuth: true,
    requiredPermissions: ['read', 'write'],
  },
  {
    path: '/admin',
    requireAuth: true,
    requiredRole: 'admin',
    requiredPermissions: ['admin'],
  },
];

// Predefined route groups
export const defaultGroups: RouteGroup[] = [
  {
    name: 'auth',
    basePath: '/auth',
    defaultAuth: false,
    routes: [
      { path: '/signin', requireAuth: false },
      { path: '/signup', requireAuth: false },
      { path: '/forgot-password', requireAuth: false },
      { path: '/reset-password', requireAuth: false },
    ],
  },
  {
    name: 'user',
    basePath: '/user',
    defaultAuth: true,
    defaultPermissions: ['read'],
    routes: [
      { path: '/profile', requiredPermissions: ['read'] },
      { path: '/settings', requiredPermissions: ['read', 'write'] },
      { path: '/preferences', requiredPermissions: ['read', 'write'] },
    ],
  },
  {
    name: 'admin',
    basePath: '/admin',
    defaultAuth: true,
    defaultRole: 'admin',
    defaultPermissions: ['admin'],
    routes: [
      { path: '/dashboard', requiredPermissions: ['admin'] },
      { path: '/users', requiredPermissions: ['admin'] },
      { path: '/settings', requiredPermissions: ['admin'] },
    ],
  },
];

// Initialize default routes
RouteManager.registerRoutes(defaultRoutes);
defaultGroups.forEach(group => RouteManager.registerGroup(group));

export default RouteManager;