/**
 * User Acceptance Testing Interface
 * Interface for Urdu speakers to participate in UAT
 */

import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Button,
  Stepper,
  Step,
  StepLabel,
  StepContent,
  TextField,
  Rating,
  Chip,
  Alert,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  Divider,
  Grid,
  Paper,
  LinearProgress,
  IconButton,
  Tooltip,
  FormControl,
  InputLabel,
  Select,
  MenuItem
} from '@mui/material';
import {
  PlayArrow,
  Stop,
  CheckCircle,
  Error,
  Warning,
  Info,
  Timer,
  Assignment,
  Feedback,
  Camera,
  Upload,
  Close
} from '@mui/icons-material';
import { useAuth } from '../../auth/AuthContext';

interface TestScenario {
  id: string;
  title: string;
  description: string;
  scenario_type: string;
  steps: string[];
  expected_outcome: string;
  acceptance_criteria: string[];
  priority: string;
  estimated_duration: number;
  prerequisites: string[];
}

interface TestExecution {
  id: string;
  scenario_id: string;
  status: string;
  started_at: string;
  completed_at?: string;
}

interface TesterProfile {
  native_language: string;
  language_proficiency: Record<string, string>;
  technical_background: string;
  device_info: Record<string, string>;
  location: string;
  availability: string;
}

export const TestingInterface: React.FC = () => {
  const { user, token } = useAuth();
  const [scenarios, setScenarios] = useState<TestScenario[]>([]);
  const [currentExecution, setCurrentExecution] = useState<TestExecution | null>(null);
  const [currentScenario, setCurrentScenario] = useState<TestScenario | null>(null);
  const [activeStep, setActiveStep] = useState(0);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isRegistered, setIsRegistered] = useState(false);
  const [registrationDialog, setRegistrationDialog] = useState(false);
  const [completionDialog, setCompletionDialog] = useState(false);
  
  // Test completion form state
  const [actualOutcome, setActualOutcome] = useState('');
  const [issuesFound, setIssuesFound] = useState<string[]>(['']);
  const [feedbackRating, setFeedbackRating] = useState<number | null>(null);
  const [feedbackComments, setFeedbackComments] = useState('');
  const [screenshots, setScreenshots] = useState<string[]>([]);
  
  // Registration form state
  const [registrationData, setRegistrationData] = useState<TesterProfile>({
    native_language: 'ur',
    language_proficiency: { ur: 'native', en: 'intermediate' },
    technical_background: 'beginner',
    device_info: { browser: '', os: '', screen_size: '' },
    location: '',
    availability: 'weekends'
  });

  useEffect(() => {
    if (token) {
      checkRegistrationStatus();
      loadTestScenarios();
    }
  }, [token]);

  const checkRegistrationStatus = async () => {
    try {
      const response = await fetch('/api/v1/multilingual/uat/my-assignments', {
        headers: { Authorization: `Bearer ${token}` }
      });
      
      if (response.ok) {
        setIsRegistered(true);
      } else if (response.status === 400) {
        setIsRegistered(false);
        setRegistrationDialog(true);
      }
    } catch (error) {
      console.error('Error checking registration status:', error);
    }
  };

  const loadTestScenarios = async () => {
    setLoading(true);
    try {
      const response = await fetch('/api/v1/multilingual/uat/scenarios', {
        headers: { Authorization: `Bearer ${token}` }
      });
      
      if (response.ok) {
        const data = await response.json();
        setScenarios(data.scenarios);
      }
    } catch (error) {
      setError('Failed to load test scenarios');
    } finally {
      setLoading(false);
    }
  };

  const registerAsTester = async () => {
    try {
      const response = await fetch('/api/v1/multilingual/uat/register-tester', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`
        },
        body: JSON.stringify(registrationData)
      });

      if (response.ok) {
        setIsRegistered(true);
        setRegistrationDialog(false);
      } else {
        const errorData = await response.json();
        setError(errorData.detail || 'Registration failed');
      }
    } catch (error) {
      setError('Registration failed');
    }
  };

  const startTest = async (scenario: TestScenario) => {
    try {
      // Create test execution
      const createResponse = await fetch('/api/v1/multilingual/uat/executions', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`
        },
        body: JSON.stringify({
          scenario_id: scenario.id,
          metadata: {
            browser: navigator.userAgent,
            screen_size: `${window.screen.width}x${window.screen.height}`,
            timestamp: new Date().toISOString()
          }
        })
      });

      if (!createResponse.ok) {
        throw new Error('Failed to create test execution');
      }

      const createData = await createResponse.json();
      
      // Start test execution
      const startResponse = await fetch(
        `/api/v1/multilingual/uat/executions/${createData.execution_id}/start`,
        {
          method: 'PUT',
          headers: { Authorization: `Bearer ${token}` }
        }
      );

      if (startResponse.ok) {
        setCurrentExecution({
          id: createData.execution_id,
          scenario_id: scenario.id,
          status: 'in_progress',
          started_at: new Date().toISOString()
        });
        setCurrentScenario(scenario);
        setActiveStep(0);
      }
    } catch (error) {
      setError('Failed to start test');
    }
  };

  const completeTest = async () => {
    if (!currentExecution || !feedbackRating) return;

    try {
      const response = await fetch(
        `/api/v1/multilingual/uat/executions/${currentExecution.id}/complete`,
        {
          method: 'PUT',
          headers: {
            'Content-Type': 'application/json',
            Authorization: `Bearer ${token}`
          },
          body: JSON.stringify({
            actual_outcome: actualOutcome,
            issues_found: issuesFound.filter(issue => issue.trim() !== ''),
            feedback_rating: feedbackRating,
            feedback_comments: feedbackComments,
            screenshots
          })
        }
      );

      if (response.ok) {
        setCurrentExecution(null);
        setCurrentScenario(null);
        setCompletionDialog(false);
        resetCompletionForm();
        // Show success message
        setError(null);
      } else {
        throw new Error('Failed to complete test');
      }
    } catch (error) {
      setError('Failed to complete test');
    }
  };

  const resetCompletionForm = () => {
    setActualOutcome('');
    setIssuesFound(['']);
    setFeedbackRating(null);
    setFeedbackComments('');
    setScreenshots([]);
  };

  const addIssueField = () => {
    setIssuesFound([...issuesFound, '']);
  };

  const updateIssue = (index: number, value: string) => {
    const newIssues = [...issuesFound];
    newIssues[index] = value;
    setIssuesFound(newIssues);
  };

  const removeIssue = (index: number) => {
    if (issuesFound.length > 1) {
      const newIssues = issuesFound.filter((_, i) => i !== index);
      setIssuesFound(newIssues);
    }
  };

  const getPriorityColor = (priority: string) => {
    switch (priority) {
      case 'high': return 'error';
      case 'medium': return 'warning';
      case 'low': return 'info';
      default: return 'default';
    }
  };

  const getScenarioTypeIcon = (type: string) => {
    switch (type) {
      case 'language_switching': return <Assignment />;
      case 'content_translation': return <Assignment />;
      case 'rtl_layout': return <Assignment />;
      case 'cultural_appropriateness': return <Assignment />;
      case 'community_features': return <Assignment />;
      case 'search_functionality': return <Assignment />;
      case 'performance': return <Timer />;
      case 'accessibility': return <Assignment />;
      default: return <Assignment />;
    }
  };

  const renderRegistrationDialog = () => (
    <Dialog open={registrationDialog} maxWidth="md" fullWidth>
      <DialogTitle>Register as UAT Tester</DialogTitle>
      <DialogContent>
        <Typography variant="body2" paragraph>
          Welcome! To participate in User Acceptance Testing for our multilingual features,
          please provide some information about yourself.
        </Typography>
        
        <Grid container spacing={2} sx={{ mt: 1 }}>
          <Grid item xs={12} sm={6}>
            <FormControl fullWidth>
              <InputLabel>Native Language</InputLabel>
              <Select
                value={registrationData.native_language}
                label="Native Language"
                onChange={(e) => setRegistrationData({
                  ...registrationData,
                  native_language: e.target.value
                })}
              >
                <MenuItem value="ur">Urdu</MenuItem>
                <MenuItem value="en">English</MenuItem>
                <MenuItem value="ar">Arabic</MenuItem>
              </Select>
            </FormControl>
          </Grid>
          
          <Grid item xs={12} sm={6}>
            <FormControl fullWidth>
              <InputLabel>Technical Background</InputLabel>
              <Select
                value={registrationData.technical_background}
                label="Technical Background"
                onChange={(e) => setRegistrationData({
                  ...registrationData,
                  technical_background: e.target.value
                })}
              >
                <MenuItem value="beginner">Beginner</MenuItem>
                <MenuItem value="intermediate">Intermediate</MenuItem>
                <MenuItem value="advanced">Advanced</MenuItem>
                <MenuItem value="expert">Expert</MenuItem>
              </Select>
            </FormControl>
          </Grid>
          
          <Grid item xs={12}>
            <TextField
              fullWidth
              label="Location"
              value={registrationData.location}
              onChange={(e) => setRegistrationData({
                ...registrationData,
                location: e.target.value
              })}
              placeholder="e.g., Karachi, Pakistan"
            />
          </Grid>
          
          <Grid item xs={12}>
            <FormControl fullWidth>
              <InputLabel>Availability</InputLabel>
              <Select
                value={registrationData.availability}
                label="Availability"
                onChange={(e) => setRegistrationData({
                  ...registrationData,
                  availability: e.target.value
                })}
              >
                <MenuItem value="weekdays">Weekdays</MenuItem>
                <MenuItem value="weekends">Weekends</MenuItem>
                <MenuItem value="evenings">Evenings</MenuItem>
                <MenuItem value="flexible">Flexible</MenuItem>
              </Select>
            </FormControl>
          </Grid>
        </Grid>
      </DialogContent>
      <DialogActions>
        <Button onClick={() => setRegistrationDialog(false)}>Cancel</Button>
        <Button variant="contained" onClick={registerAsTester}>
          Register
        </Button>
      </DialogActions>
    </Dialog>
  );

  const renderTestExecution = () => {
    if (!currentScenario) return null;

    return (
      <Card>
        <CardContent>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
            <Typography variant="h6">
              Testing: {currentScenario.title}
            </Typography>
            <Button
              variant="outlined"
              color="error"
              onClick={() => setCompletionDialog(true)}
              startIcon={<Stop />}
            >
              Complete Test
            </Button>
          </Box>
          
          <Typography variant="body2" color="text.secondary" paragraph>
            {currentScenario.description}
          </Typography>
          
          <Typography variant="subtitle2" gutterBottom>
            Expected Outcome:
          </Typography>
          <Typography variant="body2" paragraph>
            {currentScenario.expected_outcome}
          </Typography>
          
          <Stepper activeStep={activeStep} orientation="vertical">
            {currentScenario.steps.map((step, index) => (
              <Step key={index}>
                <StepLabel>Step {index + 1}</StepLabel>
                <StepContent>
                  <Typography variant="body2" paragraph>
                    {step}
                  </Typography>
                  <Box sx={{ mb: 2 }}>
                    <Button
                      variant="contained"
                      onClick={() => setActiveStep(index + 1)}
                      sx={{ mt: 1, mr: 1 }}
                      disabled={index === currentScenario.steps.length - 1}
                    >
                      {index === currentScenario.steps.length - 1 ? 'Complete' : 'Next'}
                    </Button>
                    {index > 0 && (
                      <Button
                        onClick={() => setActiveStep(index - 1)}
                        sx={{ mt: 1, mr: 1 }}
                      >
                        Back
                      </Button>
                    )}
                  </Box>
                </StepContent>
              </Step>
            ))}
          </Stepper>
          
          {activeStep === currentScenario.steps.length && (
            <Paper sx={{ p: 2, mt: 2, bgcolor: 'success.light' }}>
              <Typography variant="h6" gutterBottom>
                Test Steps Completed!
              </Typography>
              <Typography variant="body2" paragraph>
                You have completed all test steps. Please click "Complete Test" to provide your feedback.
              </Typography>
              <Button
                variant="contained"
                onClick={() => setCompletionDialog(true)}
                startIcon={<CheckCircle />}
              >
                Complete Test
              </Button>
            </Paper>
          )}
        </CardContent>
      </Card>
    );
  };

  const renderCompletionDialog = () => (
    <Dialog open={completionDialog} maxWidth="md" fullWidth>
      <DialogTitle>Complete Test Execution</DialogTitle>
      <DialogContent>
        <Grid container spacing={2}>
          <Grid item xs={12}>
            <TextField
              fullWidth
              multiline
              rows={3}
              label="Actual Outcome"
              value={actualOutcome}
              onChange={(e) => setActualOutcome(e.target.value)}
              placeholder="Describe what actually happened during the test..."
              required
            />
          </Grid>
          
          <Grid item xs={12}>
            <Typography variant="subtitle2" gutterBottom>
              Issues Found (if any)
            </Typography>
            {issuesFound.map((issue, index) => (
              <Box key={index} sx={{ display: 'flex', gap: 1, mb: 1 }}>
                <TextField
                  fullWidth
                  size="small"
                  value={issue}
                  onChange={(e) => updateIssue(index, e.target.value)}
                  placeholder={`Issue ${index + 1}...`}
                />
                {issuesFound.length > 1 && (
                  <IconButton onClick={() => removeIssue(index)}>
                    <Close />
                  </IconButton>
                )}
              </Box>
            ))}
            <Button size="small" onClick={addIssueField}>
              Add Issue
            </Button>
          </Grid>
          
          <Grid item xs={12}>
            <Typography variant="subtitle2" gutterBottom>
              Overall Rating *
            </Typography>
            <Rating
              value={feedbackRating}
              onChange={(_, newValue) => setFeedbackRating(newValue)}
              size="large"
            />
          </Grid>
          
          <Grid item xs={12}>
            <TextField
              fullWidth
              multiline
              rows={4}
              label="Detailed Feedback"
              value={feedbackComments}
              onChange={(e) => setFeedbackComments(e.target.value)}
              placeholder="Please provide detailed feedback about your testing experience..."
              required
            />
          </Grid>
        </Grid>
      </DialogContent>
      <DialogActions>
        <Button onClick={() => setCompletionDialog(false)}>Cancel</Button>
        <Button
          variant="contained"
          onClick={completeTest}
          disabled={!actualOutcome || !feedbackRating || !feedbackComments}
        >
          Submit Results
        </Button>
      </DialogActions>
    </Dialog>
  );

  if (!user) {
    return (
      <Alert severity="info">
        Please log in to participate in User Acceptance Testing.
      </Alert>
    );
  }

  if (loading) {
    return <LinearProgress />;
  }

  return (
    <Box sx={{ p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Multilingual System Testing
      </Typography>
      
      <Typography variant="body1" paragraph>
        Help us improve our multilingual features by participating in User Acceptance Testing.
        Your feedback is valuable in making the system better for Urdu speakers.
      </Typography>

      {error && (
        <Alert severity="error" sx={{ mb: 2 }}>
          {error}
        </Alert>
      )}

      {currentExecution ? (
        renderTestExecution()
      ) : (
        <Grid container spacing={3}>
          {scenarios.map((scenario) => (
            <Grid item xs={12} md={6} key={scenario.id}>
              <Card>
                <CardContent>
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
                    {getScenarioTypeIcon(scenario.scenario_type)}
                    <Typography variant="h6" noWrap>
                      {scenario.title}
                    </Typography>
                    <Chip
                      label={scenario.priority.toUpperCase()}
                      color={getPriorityColor(scenario.priority) as any}
                      size="small"
                    />
                  </Box>
                  
                  <Typography variant="body2" color="text.secondary" paragraph>
                    {scenario.description}
                  </Typography>
                  
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 2, mb: 2 }}>
                    <Chip
                      icon={<Timer />}
                      label={`${scenario.estimated_duration} min`}
                      size="small"
                      variant="outlined"
                    />
                    <Chip
                      label={scenario.scenario_type.replace(/_/g, ' ')}
                      size="small"
                      variant="outlined"
                    />
                  </Box>
                  
                  <Typography variant="subtitle2" gutterBottom>
                    Prerequisites:
                  </Typography>
                  <List dense>
                    {scenario.prerequisites.map((prereq, index) => (
                      <ListItem key={index} sx={{ py: 0 }}>
                        <ListItemIcon sx={{ minWidth: 20 }}>
                          <Info fontSize="small" />
                        </ListItemIcon>
                        <ListItemText
                          primary={prereq}
                          primaryTypographyProps={{ variant: 'body2' }}
                        />
                      </ListItem>
                    ))}
                  </List>
                  
                  <Button
                    variant="contained"
                    fullWidth
                    startIcon={<PlayArrow />}
                    onClick={() => startTest(scenario)}
                    disabled={!isRegistered}
                    sx={{ mt: 2 }}
                  >
                    Start Test
                  </Button>
                </CardContent>
              </Card>
            </Grid>
          ))}
        </Grid>
      )}

      {renderRegistrationDialog()}
      {renderCompletionDialog()}
    </Box>
  );
};

export default TestingInterface;