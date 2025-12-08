/**
 * Multilingual System Monitoring Dashboard
 * Real-time monitoring and analytics for multilingual features
 */

import React, { useState, useEffect, useCallback } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Grid,
  Alert,
  Chip,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  LinearProgress,
  IconButton,
  Tooltip,
  Select,
  MenuItem,
  FormControl,
  InputLabel,
  Button,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  Divider
} from '@mui/material';
import {
  Refresh,
  Warning,
  Error,
  CheckCircle,
  TrendingUp,
  TrendingDown,
  Speed,
  Language,
  Search,
  Translate,
  People,
  Timeline,
  Notifications,
  Close
} from '@mui/icons-material';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip as RechartsTooltip,
  ResponsiveContainer,
  BarChart,
  Bar,
  PieChart,
  Pie,
  Cell
} from 'recharts';
import { useAuth } from '../../auth/AuthContext';

interface MetricData {
  current: number;
  average: number;
  min: number;
  max: number;
  count: number;
  data_points: Array<{
    timestamp: string;
    value: number;
    tags: Record<string, string>;
  }>;
}

interface AlertData {
  id: string;
  timestamp: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  title: string;
  description: string;
  metric_name: string;
  current_value: number;
  threshold_value: number;
  resolved: boolean;
}

interface DashboardData {
  time_range: string;
  start_time: string;
  end_time: string;
  metrics: Record<string, MetricData>;
  alerts: AlertData[];
  summary: {
    total_metrics: number;
    active_alerts: number;
    critical_alerts: number;
    system_health: string;
  };
}

interface SystemHealth {
  status: string;
  timestamp: string;
  active_alerts: number;
  critical_alerts: number;
  components: Record<string, string>;
}

export const MultilingualDashboard: React.FC = () => {
  const { token } = useAuth();
  const [dashboardData, setDashboardData] = useState<DashboardData | null>(null);
  const [systemHealth, setSystemHealth] = useState<SystemHealth | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [timeRange, setTimeRange] = useState('1h');
  const [autoRefresh, setAutoRefresh] = useState(true);
  const [alertDialog, setAlertDialog] = useState<AlertData | null>(null);

  const fetchDashboardData = useCallback(async () => {
    if (!token) return;

    setLoading(true);
    setError(null);

    try {
      const response = await fetch(
        `/api/v1/multilingual/monitoring/dashboard?time_range=${timeRange}`,
        {
          headers: {
            Authorization: `Bearer ${token}`
          }
        }
      );

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`);
      }

      const data = await response.json();
      setDashboardData(data);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to fetch dashboard data');
    } finally {
      setLoading(false);
    }
  }, [token, timeRange]);

  const fetchSystemHealth = useCallback(async () => {
    if (!token) return;

    try {
      const response = await fetch('/api/v1/multilingual/monitoring/health', {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });

      if (response.ok) {
        const health = await response.json();
        setSystemHealth(health);
      }
    } catch (err) {
      console.error('Failed to fetch system health:', err);
    }
  }, [token]);

  const resolveAlert = async (alertId: string) => {
    if (!token) return;

    try {
      const response = await fetch(
        `/api/v1/multilingual/monitoring/alerts/${alertId}/resolve`,
        {
          method: 'PUT',
          headers: {
            Authorization: `Bearer ${token}`
          }
        }
      );

      if (response.ok) {
        // Refresh dashboard data
        await fetchDashboardData();
        setAlertDialog(null);
      }
    } catch (err) {
      console.error('Failed to resolve alert:', err);
    }
  };

  useEffect(() => {
    fetchDashboardData();
    fetchSystemHealth();
  }, [fetchDashboardData, fetchSystemHealth]);

  useEffect(() => {
    if (!autoRefresh) return;

    const interval = setInterval(() => {
      fetchDashboardData();
      fetchSystemHealth();
    }, 30000); // Refresh every 30 seconds

    return () => clearInterval(interval);
  }, [autoRefresh, fetchDashboardData, fetchSystemHealth]);

  const getHealthColor = (health: string) => {
    const colors: Record<string, string> = {
      excellent: '#4caf50',
      good: '#8bc34a',
      fair: '#ff9800',
      poor: '#ff5722',
      critical: '#f44336',
      unknown: '#9e9e9e'
    };
    return colors[health] || colors.unknown;
  };

  const getSeverityColor = (severity: string) => {
    const colors: Record<string, 'default' | 'primary' | 'secondary' | 'error' | 'info' | 'success' | 'warning'> = {
      low: 'info',
      medium: 'warning',
      high: 'error',
      critical: 'error'
    };
    return colors[severity] || 'default';
  };

  const getSeverityIcon = (severity: string) => {
    switch (severity) {
      case 'critical':
      case 'high':
        return <Error />;
      case 'medium':
        return <Warning />;
      default:
        return <Notifications />;
    }
  };

  const formatValue = (value: number, unit: string) => {
    if (unit === 'ms') {
      return `${value.toFixed(0)}ms`;
    } else if (unit === 's') {
      return `${value.toFixed(2)}s`;
    } else if (unit === '%') {
      return `${value.toFixed(1)}%`;
    }
    return value.toFixed(2);
  };

  const renderSystemHealthCard = () => (
    <Card>
      <CardContent>
        <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 2 }}>
          <Typography variant="h6">System Health</Typography>
          <Chip
            label={systemHealth?.status || 'Unknown'}
            sx={{
              backgroundColor: getHealthColor(systemHealth?.status || 'unknown'),
              color: 'white'
            }}
          />
        </Box>
        
        {systemHealth && (
          <Grid container spacing={2}>
            <Grid item xs={6}>
              <Typography variant="body2" color="text.secondary">
                Active Alerts: {systemHealth.active_alerts}
              </Typography>
            </Grid>
            <Grid item xs={6}>
              <Typography variant="body2" color="text.secondary">
                Critical Alerts: {systemHealth.critical_alerts}
              </Typography>
            </Grid>
          </Grid>
        )}

        {systemHealth?.components && (
          <Box sx={{ mt: 2 }}>
            <Typography variant="subtitle2" gutterBottom>
              Components
            </Typography>
            {Object.entries(systemHealth.components).map(([component, status]) => (
              <Box key={component} sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
                <CheckCircle
                  sx={{
                    color: status === 'healthy' ? 'success.main' : 
                           status === 'degraded' ? 'warning.main' : 'error.main',
                    mr: 1,
                    fontSize: 16
                  }}
                />
                <Typography variant="body2">
                  {component.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}: {status}
                </Typography>
              </Box>
            ))}
          </Box>
        )}
      </CardContent>
    </Card>
  );

  const renderMetricCard = (name: string, data: MetricData, unit: string) => {
    const isPerformanceMetric = name.includes('time') || name.includes('response');
    const trend = data.current > data.average ? 'up' : 'down';
    const trendColor = isPerformanceMetric 
      ? (trend === 'up' ? 'error' : 'success')
      : (trend === 'up' ? 'success' : 'error');

    return (
      <Card key={name}>
        <CardContent>
          <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between', mb: 1 }}>
            <Typography variant="h6" noWrap>
              {name.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}
            </Typography>
            {trend === 'up' ? (
              <TrendingUp color={trendColor} />
            ) : (
              <TrendingDown color={trendColor} />
            )}
          </Box>
          
          <Typography variant="h4" color="primary" gutterBottom>
            {formatValue(data.current, unit)}
          </Typography>
          
          <Grid container spacing={1}>
            <Grid item xs={4}>
              <Typography variant="body2" color="text.secondary">
                Avg: {formatValue(data.average, unit)}
              </Typography>
            </Grid>
            <Grid item xs={4}>
              <Typography variant="body2" color="text.secondary">
                Min: {formatValue(data.min, unit)}
              </Typography>
            </Grid>
            <Grid item xs={4}>
              <Typography variant="body2" color="text.secondary">
                Max: {formatValue(data.max, unit)}
              </Typography>
            </Grid>
          </Grid>
          
          <Typography variant="caption" color="text.secondary">
            {data.count} samples
          </Typography>
        </CardContent>
      </Card>
    );
  };

  const renderMetricChart = (name: string, data: MetricData, unit: string) => {
    const chartData = data.data_points.map(point => ({
      timestamp: new Date(point.timestamp).toLocaleTimeString(),
      value: point.value
    }));

    return (
      <Card sx={{ mt: 2 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>
            {name.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())} Trend
          </Typography>
          <ResponsiveContainer width="100%" height={200}>
            <LineChart data={chartData}>
              <CartesianGrid strokeDasharray="3 3" />
              <XAxis dataKey="timestamp" />
              <YAxis />
              <RechartsTooltip 
                formatter={(value: number) => [formatValue(value, unit), name]}
              />
              <Line 
                type="monotone" 
                dataKey="value" 
                stroke="#1976d2" 
                strokeWidth={2}
                dot={false}
              />
            </LineChart>
          </ResponsiveContainer>
        </CardContent>
      </Card>
    );
  };

  const renderAlertsTable = () => (
    <Card>
      <CardContent>
        <Typography variant="h6" gutterBottom>
          Active Alerts
        </Typography>
        
        {dashboardData?.alerts.length === 0 ? (
          <Alert severity="success">No active alerts</Alert>
        ) : (
          <TableContainer>
            <Table>
              <TableHead>
                <TableRow>
                  <TableCell>Severity</TableCell>
                  <TableCell>Title</TableCell>
                  <TableCell>Metric</TableCell>
                  <TableCell>Value</TableCell>
                  <TableCell>Time</TableCell>
                  <TableCell>Actions</TableCell>
                </TableRow>
              </TableHead>
              <TableBody>
                {dashboardData?.alerts.map((alert) => (
                  <TableRow key={alert.id}>
                    <TableCell>
                      <Chip
                        icon={getSeverityIcon(alert.severity)}
                        label={alert.severity.toUpperCase()}
                        color={getSeverityColor(alert.severity)}
                        size="small"
                      />
                    </TableCell>
                    <TableCell>{alert.title}</TableCell>
                    <TableCell>{alert.metric_name}</TableCell>
                    <TableCell>
                      {alert.current_value.toFixed(2)} / {alert.threshold_value.toFixed(2)}
                    </TableCell>
                    <TableCell>
                      {new Date(alert.timestamp).toLocaleString()}
                    </TableCell>
                    <TableCell>
                      <Button
                        size="small"
                        onClick={() => setAlertDialog(alert)}
                      >
                        View
                      </Button>
                    </TableCell>
                  </TableRow>
                ))}
              </TableBody>
            </Table>
          </TableContainer>
        )}
      </CardContent>
    </Card>
  );

  if (!token) {
    return (
      <Alert severity="warning">
        Please log in with admin privileges to access the monitoring dashboard.
      </Alert>
    );
  }

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Typography variant="h4">
          Multilingual System Monitoring
        </Typography>
        
        <Box sx={{ display: 'flex', gap: 2, alignItems: 'center' }}>
          <FormControl size="small" sx={{ minWidth: 120 }}>
            <InputLabel>Time Range</InputLabel>
            <Select
              value={timeRange}
              label="Time Range"
              onChange={(e) => setTimeRange(e.target.value)}
            >
              <MenuItem value="5m">5 minutes</MenuItem>
              <MenuItem value="15m">15 minutes</MenuItem>
              <MenuItem value="1h">1 hour</MenuItem>
              <MenuItem value="6h">6 hours</MenuItem>
              <MenuItem value="24h">24 hours</MenuItem>
            </Select>
          </FormControl>
          
          <Button
            variant="outlined"
            onClick={() => setAutoRefresh(!autoRefresh)}
            color={autoRefresh ? 'primary' : 'inherit'}
          >
            Auto Refresh: {autoRefresh ? 'ON' : 'OFF'}
          </Button>
          
          <Tooltip title="Refresh Now">
            <IconButton onClick={fetchDashboardData} disabled={loading}>
              <Refresh />
            </IconButton>
          </Tooltip>
        </Box>
      </Box>

      {loading && <LinearProgress sx={{ mb: 2 }} />}
      
      {error && (
        <Alert severity="error" sx={{ mb: 2 }}>
          {error}
        </Alert>
      )}

      <Grid container spacing={3}>
        {/* System Health */}
        <Grid item xs={12} md={4}>
          {renderSystemHealthCard()}
        </Grid>

        {/* Key Metrics */}
        {dashboardData?.metrics && Object.entries(dashboardData.metrics).map(([name, data]) => {
          const unit = name.includes('time') ? 's' : 
                      name.includes('ratio') ? '%' : 
                      name.includes('response') ? 'ms' : '';
          
          return (
            <Grid item xs={12} md={4} key={name}>
              {renderMetricCard(name, data, unit)}
            </Grid>
          );
        })}

        {/* Alerts Table */}
        <Grid item xs={12}>
          {renderAlertsTable()}
        </Grid>

        {/* Metric Charts */}
        {dashboardData?.metrics && Object.entries(dashboardData.metrics).map(([name, data]) => {
          const unit = name.includes('time') ? 's' : 
                      name.includes('ratio') ? '%' : 
                      name.includes('response') ? 'ms' : '';
          
          return (
            <Grid item xs={12} md={6} key={`chart-${name}`}>
              {renderMetricChart(name, data, unit)}
            </Grid>
          );
        })}
      </Grid>

      {/* Alert Detail Dialog */}
      <Dialog
        open={!!alertDialog}
        onClose={() => setAlertDialog(null)}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>
          <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
            Alert Details
            <IconButton onClick={() => setAlertDialog(null)}>
              <Close />
            </IconButton>
          </Box>
        </DialogTitle>
        
        <DialogContent>
          {alertDialog && (
            <Box>
              <Grid container spacing={2}>
                <Grid item xs={12} sm={6}>
                  <Typography variant="subtitle2">Severity</Typography>
                  <Chip
                    icon={getSeverityIcon(alertDialog.severity)}
                    label={alertDialog.severity.toUpperCase()}
                    color={getSeverityColor(alertDialog.severity)}
                  />
                </Grid>
                
                <Grid item xs={12} sm={6}>
                  <Typography variant="subtitle2">Timestamp</Typography>
                  <Typography variant="body2">
                    {new Date(alertDialog.timestamp).toLocaleString()}
                  </Typography>
                </Grid>
                
                <Grid item xs={12}>
                  <Typography variant="subtitle2">Description</Typography>
                  <Typography variant="body2">
                    {alertDialog.description}
                  </Typography>
                </Grid>
                
                <Grid item xs={12} sm={6}>
                  <Typography variant="subtitle2">Current Value</Typography>
                  <Typography variant="body2">
                    {alertDialog.current_value.toFixed(2)}
                  </Typography>
                </Grid>
                
                <Grid item xs={12} sm={6}>
                  <Typography variant="subtitle2">Threshold</Typography>
                  <Typography variant="body2">
                    {alertDialog.threshold_value.toFixed(2)}
                  </Typography>
                </Grid>
              </Grid>
            </Box>
          )}
        </DialogContent>
        
        <DialogActions>
          <Button onClick={() => setAlertDialog(null)}>
            Close
          </Button>
          {alertDialog && !alertDialog.resolved && (
            <Button
              variant="contained"
              onClick={() => resolveAlert(alertDialog.id)}
            >
              Resolve Alert
            </Button>
          )}
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default MultilingualDashboard;