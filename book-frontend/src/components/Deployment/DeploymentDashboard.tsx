/**
 * Deployment Dashboard for Multilingual System
 * Manages gradual rollout and feature flags
 */

import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Button,
  Grid,
  Chip,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  LinearProgress,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  TextField,
  FormControl,
  InputLabel,
  Select,
  MenuItem,
  Switch,
  FormControlLabel,
  Alert,
  IconButton,
  Tooltip,
  Stepper,
  Step,
  StepLabel,
  StepContent
} from '@mui/material';
import {
  PlayArrow,
  Stop,
  Refresh,
  Warning,
  CheckCircle,
  Error,
  Timeline,
  Flag,
  People,
  TrendingUp,
  Settings,
  Visibility
} from '@mui/icons-material';
import { useAuth } from '../../auth/AuthContext';

interface Deployment {
  id: string;
  version: string;
  stage: string;
  status: string;
  started_at: string;
  completed_at?: string;
  percentage: number;
}

interface DeploymentHealth {
  deployment_id: string;
  stage: string;
  status: string;
  metrics: Record<string, number>;
  success_criteria_met: boolean;
  rollback_needed: boolean;
  duration_hours: number;
  max_duration_hours: number;
  action_taken?: string;
}

interface FeatureFlags {
  [key: string]: boolean;
}

interface RolloutConfig {
  percentage: number;
  duration_hours: number;
  success_criteria: Record<string, number>;
  rollback_criteria: Record<string, number>;
  user_segments: string[];
  feature_flags: string[];
}

export const DeploymentDashboard: React.FC = () => {
  const { token } = useAuth();
  const [deployments, setDeployments] = useState<Deployment[]>([]);
  const [featureFlags, setFeatureFlags] = useState<FeatureFlags>({});
  const [rolloutConfigs, setRolloutConfigs] = useState<Record<string, RolloutConfig>>({});
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  
  // Dialog states
  const [deployDialog, setDeployDialog] = useState(false);
  const [healthDialog, setHealthDialog] = useState<DeploymentHealth | null>(null);
  const [flagDialog, setFlagDialog] = useState<string | null>(null);
  
  // Form states
  const [newVersion, setNewVersion] = useState('');
  const [selectedStage, setSelectedStage] = useState('internal');
  const [flagEnabled, setFlagEnabled] = useState(false);
  const [flagPercentage, setFlagPercentage] = useState(100);

  useEffect(() => {
    if (token) {
      loadData();
    }
  }, [token]);

  const loadData = async () => {
    setLoading(true);
    try {
      await Promise.all([
        loadDeployments(),
        loadFeatureFlags(),
        loadRolloutConfigs()
      ]);
    } catch (error) {
      setError('Failed to load dashboard data');
    } finally {
      setLoading(false);
    }
  };

  const loadDeployments = async () => {
    try {
      const response = await fetch('/api/v1/multilingual/deployment/list', {
        headers: { Authorization: `Bearer ${token}` }
      });
      
      if (response.ok) {
        const data = await response.json();
        setDeployments(data.deployments);
      }
    } catch (error) {
      console.error('Error loading deployments:', error);
    }
  };

  const loadFeatureFlags = async () => {
    try {
      const response = await fetch('/api/v1/multilingual/deployment/feature-flags', {
        headers: { Authorization: `Bearer ${token}` }
      });
      
      if (response.ok) {
        const data = await response.json();
        setFeatureFlags(data.feature_flags);
      }
    } catch (error) {
      console.error('Error loading feature flags:', error);
    }
  };

  const loadRolloutConfigs = async () => {
    try {
      const response = await fetch('/api/v1/multilingual/deployment/rollout-config', {
        headers: { Authorization: `Bearer ${token}` }
      });
      
      if (response.ok) {
        const data = await response.json();
        setRolloutConfigs(data.rollout_configs);
      }
    } catch (error) {
      console.error('Error loading rollout configs:', error);
    }
  };

  const startDeployment = async () => {
    try {
      const response = await fetch('/api/v1/multilingual/deployment/start', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`
        },
        body: JSON.stringify({
          version: newVersion,
          stage: selectedStage
        })
      });

      if (response.ok) {
        setDeployDialog(false);
        setNewVersion('');
        await loadDeployments();
      } else {
        const errorData = await response.json();
        setError(errorData.detail || 'Failed to start deployment');
      }
    } catch (error) {
      setError('Failed to start deployment');
    }
  };

  const checkDeploymentHealth = async (deploymentId: string) => {
    try {
      const response = await fetch(
        `/api/v1/multilingual/deployment/health/${deploymentId}`,
        {
          headers: { Authorization: `Bearer ${token}` }
        }
      );

      if (response.ok) {
        const health = await response.json();
        setHealthDialog(health);
      }
    } catch (error) {
      console.error('Error checking deployment health:', error);
    }
  };

  const rollbackDeployment = async (deploymentId: string, reason: string) => {
    try {
      const response = await fetch(
        `/api/v1/multilingual/deployment/rollback/${deploymentId}?reason=${encodeURIComponent(reason)}`,
        {
          method: 'POST',
          headers: { Authorization: `Bearer ${token}` }
        }
      );

      if (response.ok) {
        await loadDeployments();
        setHealthDialog(null);
      }
    } catch (error) {
      console.error('Error rolling back deployment:', error);
    }
  };

  const updateFeatureFlag = async (feature: string, enabled: boolean, percentage?: number) => {
    try {
      const response = await fetch('/api/v1/multilingual/deployment/feature-flags', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`
        },
        body: JSON.stringify({
          feature,
          enabled,
          percentage: percentage || 100
        })
      });

      if (response.ok) {
        await loadFeatureFlags();
        setFlagDialog(null);
      }
    } catch (error) {
      console.error('Error updating feature flag:', error);
    }
  };

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'completed': return 'success';
      case 'in_progress': return 'primary';
      case 'failed': return 'error';
      case 'rolled_back': return 'warning';
      default: return 'default';
    }
  };

  const getStageIcon = (stage: string) => {
    switch (stage) {
      case 'internal': return <Settings />;
      case 'beta': return <People />;
      case 'gradual': return <TrendingUp />;
      case 'full': return <CheckCircle />;
      default: return <Flag />;
    }
  };

  const renderDeploymentTable = () => (
    <TableContainer component={Paper}>
      <Table>
        <TableHead>
          <TableRow>
            <TableCell>Version</TableCell>
            <TableCell>Stage</TableCell>
            <TableCell>Status</TableCell>
            <TableCell>Percentage</TableCell>
            <TableCell>Started</TableCell>
            <TableCell>Actions</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {deployments.map((deployment) => (
            <TableRow key={deployment.id}>
              <TableCell>{deployment.version}</TableCell>
              <TableCell>
                <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                  {getStageIcon(deployment.stage)}
                  {deployment.stage.toUpperCase()}
                </Box>
              </TableCell>
              <TableCell>
                <Chip
                  label={deployment.status.toUpperCase()}
                  color={getStatusColor(deployment.status) as any}
                  size="small"
                />
              </TableCell>
              <TableCell>{deployment.percentage}%</TableCell>
              <TableCell>
                {new Date(deployment.started_at).toLocaleString()}
              </TableCell>
              <TableCell>
                <Tooltip title="Check Health">
                  <IconButton
                    onClick={() => checkDeploymentHealth(deployment.id)}
                    disabled={deployment.status !== 'in_progress'}
                  >
                    <Visibility />
                  </IconButton>
                </Tooltip>
              </TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );

  const renderFeatureFlagsTable = () => (
    <TableContainer component={Paper}>
      <Table>
        <TableHead>
          <TableRow>
            <TableCell>Feature</TableCell>
            <TableCell>Status</TableCell>
            <TableCell>Actions</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {Object.entries(featureFlags).map(([feature, enabled]) => (
            <TableRow key={feature}>
              <TableCell>
                <Typography variant="body2">
                  {feature.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}
                </Typography>
              </TableCell>
              <TableCell>
                <Chip
                  label={enabled ? 'ENABLED' : 'DISABLED'}
                  color={enabled ? 'success' : 'default'}
                  size="small"
                />
              </TableCell>
              <TableCell>
                <Button
                  size="small"
                  onClick={() => {
                    setFlagDialog(feature);
                    setFlagEnabled(enabled);
                  }}
                >
                  Configure
                </Button>
              </TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );

  const renderRolloutStages = () => {
    const stages = ['internal', 'beta', 'gradual', 'full'];
    
    return (
      <Stepper orientation="vertical">
        {stages.map((stage) => {
          const config = rolloutConfigs[stage];
          if (!config) return null;

          return (
            <Step key={stage} active={true}>
              <StepLabel>
                <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                  {getStageIcon(stage)}
                  {stage.toUpperCase()}
                  <Chip label={`${config.percentage}%`} size="small" />
                </Box>
              </StepLabel>
              <StepContent>
                <Typography variant="body2" paragraph>
                  Duration: {config.duration_hours} hours
                </Typography>
                <Typography variant="body2" paragraph>
                  User Segments: {config.user_segments.join(', ')}
                </Typography>
                <Typography variant="body2" paragraph>
                  Features: {config.feature_flags.length} enabled
                </Typography>
              </StepContent>
            </Step>
          );
        })}
      </Stepper>
    );
  };

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Typography variant="h4">
          Deployment Dashboard
        </Typography>
        <Box sx={{ display: 'flex', gap: 2 }}>
          <Button
            variant="contained"
            startIcon={<PlayArrow />}
            onClick={() => setDeployDialog(true)}
          >
            New Deployment
          </Button>
          <Button
            variant="outlined"
            startIcon={<Refresh />}
            onClick={loadData}
            disabled={loading}
          >
            Refresh
          </Button>
        </Box>
      </Box>

      {loading && <LinearProgress sx={{ mb: 2 }} />}
      
      {error && (
        <Alert severity="error" sx={{ mb: 2 }}>
          {error}
        </Alert>
      )}

      <Grid container spacing={3}>
        {/* Deployments */}
        <Grid item xs={12}>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Active Deployments
              </Typography>
              {deployments.length > 0 ? (
                renderDeploymentTable()
              ) : (
                <Alert severity="info">No deployments found</Alert>
              )}
            </CardContent>
          </Card>
        </Grid>

        {/* Feature Flags */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Feature Flags
              </Typography>
              {renderFeatureFlagsTable()}
            </CardContent>
          </Card>
        </Grid>

        {/* Rollout Configuration */}
        <Grid item xs={12} md={6}>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Rollout Stages
              </Typography>
              {renderRolloutStages()}
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* New Deployment Dialog */}
      <Dialog open={deployDialog} onClose={() => setDeployDialog(false)}>
        <DialogTitle>Start New Deployment</DialogTitle>
        <DialogContent>
          <Grid container spacing={2} sx={{ mt: 1 }}>
            <Grid item xs={12}>
              <TextField
                fullWidth
                label="Version"
                value={newVersion}
                onChange={(e) => setNewVersion(e.target.value)}
                placeholder="e.g., v1.2.0"
              />
            </Grid>
            <Grid item xs={12}>
              <FormControl fullWidth>
                <InputLabel>Initial Stage</InputLabel>
                <Select
                  value={selectedStage}
                  label="Initial Stage"
                  onChange={(e) => setSelectedStage(e.target.value)}
                >
                  <MenuItem value="internal">Internal</MenuItem>
                  <MenuItem value="beta">Beta</MenuItem>
                  <MenuItem value="gradual">Gradual</MenuItem>
                  <MenuItem value="full">Full</MenuItem>
                </Select>
              </FormControl>
            </Grid>
          </Grid>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setDeployDialog(false)}>Cancel</Button>
          <Button
            variant="contained"
            onClick={startDeployment}
            disabled={!newVersion}
          >
            Start Deployment
          </Button>
        </DialogActions>
      </Dialog>

      {/* Health Check Dialog */}
      <Dialog
        open={!!healthDialog}
        onClose={() => setHealthDialog(null)}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>Deployment Health Check</DialogTitle>
        <DialogContent>
          {healthDialog && (
            <Grid container spacing={2}>
              <Grid item xs={12}>
                <Typography variant="h6" gutterBottom>
                  Stage: {healthDialog.stage.toUpperCase()}
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  Duration: {healthDialog.duration_hours.toFixed(1)} / {healthDialog.max_duration_hours} hours
                </Typography>
              </Grid>
              
              <Grid item xs={12}>
                <Typography variant="subtitle2" gutterBottom>
                  Current Metrics
                </Typography>
                {Object.entries(healthDialog.metrics).map(([metric, value]) => (
                  <Box key={metric} sx={{ display: 'flex', justifyContent: 'space-between', mb: 1 }}>
                    <Typography variant="body2">
                      {metric.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}:
                    </Typography>
                    <Typography variant="body2" sx={{ fontWeight: 'bold' }}>
                      {typeof value === 'number' ? value.toFixed(2) : value}
                    </Typography>
                  </Box>
                ))}
              </Grid>
              
              <Grid item xs={12}>
                <Box sx={{ display: 'flex', gap: 2 }}>
                  <Chip
                    icon={healthDialog.success_criteria_met ? <CheckCircle /> : <Warning />}
                    label={healthDialog.success_criteria_met ? 'Success Criteria Met' : 'Success Criteria Not Met'}
                    color={healthDialog.success_criteria_met ? 'success' : 'warning'}
                  />
                  <Chip
                    icon={healthDialog.rollback_needed ? <Error /> : <CheckCircle />}
                    label={healthDialog.rollback_needed ? 'Rollback Needed' : 'Healthy'}
                    color={healthDialog.rollback_needed ? 'error' : 'success'}
                  />
                </Box>
              </Grid>
              
              {healthDialog.action_taken && (
                <Grid item xs={12}>
                  <Alert severity="info">
                    Action taken: {healthDialog.action_taken}
                  </Alert>
                </Grid>
              )}
            </Grid>
          )}
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setHealthDialog(null)}>Close</Button>
          {healthDialog?.rollback_needed && (
            <Button
              variant="contained"
              color="error"
              onClick={() => rollbackDeployment(healthDialog.deployment_id, 'Manual rollback due to health check')}
            >
              Rollback
            </Button>
          )}
        </DialogActions>
      </Dialog>

      {/* Feature Flag Dialog */}
      <Dialog open={!!flagDialog} onClose={() => setFlagDialog(null)}>
        <DialogTitle>Configure Feature Flag</DialogTitle>
        <DialogContent>
          {flagDialog && (
            <Grid container spacing={2} sx={{ mt: 1 }}>
              <Grid item xs={12}>
                <Typography variant="h6">
                  {flagDialog.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}
                </Typography>
              </Grid>
              <Grid item xs={12}>
                <FormControlLabel
                  control={
                    <Switch
                      checked={flagEnabled}
                      onChange={(e) => setFlagEnabled(e.target.checked)}
                    />
                  }
                  label="Enabled"
                />
              </Grid>
              {flagEnabled && (
                <Grid item xs={12}>
                  <TextField
                    fullWidth
                    type="number"
                    label="Percentage of Users"
                    value={flagPercentage}
                    onChange={(e) => setFlagPercentage(Number(e.target.value))}
                    inputProps={{ min: 0, max: 100 }}
                  />
                </Grid>
              )}
            </Grid>
          )}
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setFlagDialog(null)}>Cancel</Button>
          <Button
            variant="contained"
            onClick={() => updateFeatureFlag(flagDialog!, flagEnabled, flagPercentage)}
          >
            Update
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default DeploymentDashboard;