/**
 * Personalized User Experience for Multilingual Features
 * Adapts interface based on user behavior and feedback
 */

import React, { useState, useEffect, useCallback } from 'react';
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
  Dialog,
  DialogTitle,
  DialogContent,
  DialogActions,
  FormControl,
  FormControlLabel,
  RadioGroup,
  Radio,
  Checkbox,
  Slider,
  TextField,
  Chip,
  Alert,
  Grid,
  Paper,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  Divider,
  Tooltip,
  IconButton,
  Switch
} from '@mui/material';
import {
  Language,
  School,
  Speed,
  Visibility,
  VolumeUp,
  Palette,
  Settings,
  TrendingUp,
  Psychology,
  Star,
  Close,
  CheckCircle,
  Lightbulb
} from '@mui/icons-material';
import { useAuth } from '../../auth/AuthContext';

interface UserPreferences {
  language_proficiency: {
    native: string;
    secondary: string[];
    learning: string[];
  };
  interface_preferences: {
    font_size: number;
    contrast_mode: string;
    animation_speed: string;
    sound_enabled: boolean;
  };
  learning_style: {
    visual_learner: boolean;
    audio_learner: boolean;
    kinesthetic_learner: boolean;
    reading_preference: string;
  };
  translation_preferences: {
    show_alternatives: boolean;
    confidence_threshold: number;
    prefer_human_translations: boolean;
    cultural_adaptation_level: string;
  };
  onboarding_completed: boolean;
  feedback_frequency: string;
}

interface LearningInsight {
  type: string;
  title: string;
  description: string;
  confidence: number;
  actionable: boolean;
  implemented: boolean;
}

export const PersonalizedExperience: React.FC = () => {
  const { user, token } = useAuth();
  const [preferences, setPreferences] = useState<UserPreferences | null>(null);
  const [onboardingOpen, setOnboardingOpen] = useState(false);
  const [activeStep, setActiveStep] = useState(0);
  const [insights, setInsights] = useState<LearningInsight[]>([]);
  const [feedbackDialog, setFeedbackDialog] = useState(false);
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    if (user && token) {
      loadUserPreferences();
      loadLearningInsights();
    }
  }, [user, token]);

  const loadUserPreferences = async () => {
    try {
      const response = await fetch('/api/v1/multilingual/user/preferences', {
        headers: { Authorization: `Bearer ${token}` }
      });
      
      if (response.ok) {
        const data = await response.json();
        setPreferences(data);
        
        // Show onboarding if not completed
        if (!data.onboarding_completed) {
          setOnboardingOpen(true);
        }
      }
    } catch (error) {
      console.error('Error loading preferences:', error);
    }
  };

  const loadLearningInsights = async () => {
    try {
      const response = await fetch('/api/v1/multilingual/user/insights', {
        headers: { Authorization: `Bearer ${token}` }
      });
      
      if (response.ok) {
        const data = await response.json();
        setInsights(data.insights || []);
      }
    } catch (error) {
      console.error('Error loading insights:', error);
    }
  };

  const updatePreferences = async (newPreferences: Partial<UserPreferences>) => {
    try {
      const response = await fetch('/api/v1/multilingual/user/preferences', {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          Authorization: `Bearer ${token}`
        },
        body: JSON.stringify(newPreferences)
      });

      if (response.ok) {
        setPreferences(prev => ({ ...prev, ...newPreferences } as UserPreferences));
      }
    } catch (error) {
      console.error('Error updating preferences:', error);
    }
  };

  const completeOnboarding = async () => {
    await updatePreferences({ onboarding_completed: true });
    setOnboardingOpen(false);
  };

  const renderOnboardingStep = (step: number) => {
    switch (step) {
      case 0:
        return (
          <Box>
            <Typography variant="h6" gutterBottom>
              Welcome to Multilingual Learning!
            </Typography>
            <Typography variant="body2" paragraph>
              Let's personalize your experience. First, tell us about your language background.
            </Typography>
            
            <FormControl component="fieldset" sx={{ mt: 2 }}>
              <Typography variant="subtitle2" gutterBottom>
                What's your native language?
              </Typography>
              <RadioGroup
                value={preferences?.language_proficiency.native || ''}
                onChange={(e) => updatePreferences({
                  language_proficiency: {
                    ...preferences?.language_proficiency,
                    native: e.target.value
                  }
                } as any)}
              >
                <FormControlLabel value="ur" control={<Radio />} label="Urdu" />
                <FormControlLabel value="en" control={<Radio />} label="English" />
                <FormControlLabel value="ar" control={<Radio />} label="Arabic" />
                <FormControlLabel value="other" control={<Radio />} label="Other" />
              </RadioGroup>
            </FormControl>
          </Box>
        );

      case 1:
        return (
          <Box>
            <Typography variant="h6" gutterBottom>
              Learning Style Assessment
            </Typography>
            <Typography variant="body2" paragraph>
              Help us understand how you learn best.
            </Typography>
            
            <Grid container spacing={2}>
              <Grid item xs={12}>
                <FormControlLabel
                  control={
                    <Checkbox
                      checked={preferences?.learning_style.visual_learner || false}
                      onChange={(e) => updatePreferences({
                        learning_style: {
                          ...preferences?.learning_style,
                          visual_learner: e.target.checked
                        }
                      } as any)}
                    />
                  }
                  label="I learn better with visual aids (images, diagrams, colors)"
                />
              </Grid>
              <Grid item xs={12}>
                <FormControlLabel
                  control={
                    <Checkbox
                      checked={preferences?.learning_style.audio_learner || false}
                      onChange={(e) => updatePreferences({
                        learning_style: {
                          ...preferences?.learning_style,
                          audio_learner: e.target.checked
                        }
                      } as any)}
                    />
                  }
                  label="I learn better with audio (pronunciation, explanations)"
                />
              </Grid>
              <Grid item xs={12}>
                <FormControlLabel
                  control={
                    <Checkbox
                      checked={preferences?.learning_style.kinesthetic_learner || false}
                      onChange={(e) => updatePreferences({
                        learning_style: {
                          ...preferences?.learning_style,
                          kinesthetic_learner: e.target.checked
                        }
                      } as any)}
                    />
                  }
                  label="I learn better through practice and interaction"
                />
              </Grid>
            </Grid>
          </Box>
        );

      case 2:
        return (
          <Box>
            <Typography variant="h6" gutterBottom>
              Interface Preferences
            </Typography>
            <Typography variant="body2" paragraph>
              Customize the interface to your comfort.
            </Typography>
            
            <Grid container spacing={3}>
              <Grid item xs={12}>
                <Typography variant="subtitle2" gutterBottom>
                  Font Size
                </Typography>
                <Slider
                  value={preferences?.interface_preferences.font_size || 16}
                  onChange={(_, value) => updatePreferences({
                    interface_preferences: {
                      ...preferences?.interface_preferences,
                      font_size: value as number
                    }
                  } as any)}
                  min={12}
                  max={24}
                  step={1}
                  marks
                  valueLabelDisplay="auto"
                />
              </Grid>
              
              <Grid item xs={12}>
                <FormControl component="fieldset">
                  <Typography variant="subtitle2" gutterBottom>
                    Contrast Mode
                  </Typography>
                  <RadioGroup
                    value={preferences?.interface_preferences.contrast_mode || 'normal'}
                    onChange={(e) => updatePreferences({
                      interface_preferences: {
                        ...preferences?.interface_preferences,
                        contrast_mode: e.target.value
                      }
                    } as any)}
                  >
                    <FormControlLabel value="normal" control={<Radio />} label="Normal" />
                    <FormControlLabel value="high" control={<Radio />} label="High Contrast" />
                    <FormControlLabel value="dark" control={<Radio />} label="Dark Mode" />
                  </RadioGroup>
                </FormControl>
              </Grid>
            </Grid>
          </Box>
        );

      case 3:
        return (
          <Box>
            <Typography variant="h6" gutterBottom>
              Translation Preferences
            </Typography>
            <Typography variant="body2" paragraph>
              Configure how translations are presented to you.
            </Typography>
            
            <Grid container spacing={2}>
              <Grid item xs={12}>
                <FormControlLabel
                  control={
                    <Switch
                      checked={preferences?.translation_preferences.show_alternatives || false}
                      onChange={(e) => updatePreferences({
                        translation_preferences: {
                          ...preferences?.translation_preferences,
                          show_alternatives: e.target.checked
                        }
                      } as any)}
                    />
                  }
                  label="Show alternative translations"
                />
              </Grid>
              
              <Grid item xs={12}>
                <Typography variant="subtitle2" gutterBottom>
                  Translation Confidence Threshold
                </Typography>
                <Slider
                  value={preferences?.translation_preferences.confidence_threshold || 0.8}
                  onChange={(_, value) => updatePreferences({
                    translation_preferences: {
                      ...preferences?.translation_preferences,
                      confidence_threshold: value as number
                    }
                  } as any)}
                  min={0.5}
                  max={1.0}
                  step={0.1}
                  marks
                  valueLabelDisplay="auto"
                />
              </Grid>
              
              <Grid item xs={12}>
                <FormControl component="fieldset">
                  <Typography variant="subtitle2" gutterBottom>
                    Cultural Adaptation Level
                  </Typography>
                  <RadioGroup
                    value={preferences?.translation_preferences.cultural_adaptation_level || 'moderate'}
                    onChange={(e) => updatePreferences({
                      translation_preferences: {
                        ...preferences?.translation_preferences,
                        cultural_adaptation_level: e.target.value
                      }
                    } as any)}
                  >
                    <FormControlLabel value="minimal" control={<Radio />} label="Minimal - Keep original context" />
                    <FormControlLabel value="moderate" control={<Radio />} label="Moderate - Some local adaptation" />
                    <FormControlLabel value="full" control={<Radio />} label="Full - Complete cultural localization" />
                  </RadioGroup>
                </FormControl>
              </Grid>
            </Grid>
          </Box>
        );

      default:
        return null;
    }
  };

  const renderLearningInsights = () => (
    <Card sx={{ mt: 3 }}>
      <CardContent>
        <Typography variant="h6" gutterBottom>
          <Psychology sx={{ mr: 1, verticalAlign: 'middle' }} />
          Learning Insights
        </Typography>
        <Typography variant="body2" color="text.secondary" paragraph>
          Based on your usage patterns, here are some personalized recommendations.
        </Typography>
        
        {insights.length === 0 ? (
          <Alert severity="info">
            Keep using the platform to get personalized insights!
          </Alert>
        ) : (
          <List>
            {insights.map((insight, index) => (
              <React.Fragment key={index}>
                <ListItem>
                  <ListItemIcon>
                    {insight.type === 'improvement' ? <TrendingUp color="success" /> : <Lightbulb color="primary" />}
                  </ListItemIcon>
                  <ListItemText
                    primary={insight.title}
                    secondary={
                      <Box>
                        <Typography variant="body2" paragraph>
                          {insight.description}
                        </Typography>
                        <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                          <Chip
                            label={`${Math.round(insight.confidence * 100)}% confidence`}
                            size="small"
                            color={insight.confidence > 0.8 ? 'success' : 'default'}
                          />
                          {insight.actionable && (
                            <Chip
                              label="Actionable"
                              size="small"
                              color="primary"
                              variant="outlined"
                            />
                          )}
                          {insight.implemented && (
                            <Chip
                              label="Applied"
                              size="small"
                              color="success"
                              icon={<CheckCircle />}
                            />
                          )}
                        </Box>
                      </Box>
                    }
                  />
                </ListItem>
                {index < insights.length - 1 && <Divider />}
              </React.Fragment>
            ))}
          </List>
        )}
      </CardContent>
    </Card>
  );

  const renderQuickSettings = () => (
    <Card>
      <CardContent>
        <Typography variant="h6" gutterBottom>
          <Settings sx={{ mr: 1, verticalAlign: 'middle' }} />
          Quick Settings
        </Typography>
        
        <Grid container spacing={2}>
          <Grid item xs={12} sm={6}>
            <FormControlLabel
              control={
                <Switch
                  checked={preferences?.interface_preferences.sound_enabled || false}
                  onChange={(e) => updatePreferences({
                    interface_preferences: {
                      ...preferences?.interface_preferences,
                      sound_enabled: e.target.checked
                    }
                  } as any)}
                />
              }
              label="Sound Effects"
            />
          </Grid>
          
          <Grid item xs={12} sm={6}>
            <FormControlLabel
              control={
                <Switch
                  checked={preferences?.translation_preferences.show_alternatives || false}
                  onChange={(e) => updatePreferences({
                    translation_preferences: {
                      ...preferences?.translation_preferences,
                      show_alternatives: e.target.checked
                    }
                  } as any)}
                />
              }
              label="Show Translation Alternatives"
            />
          </Grid>
          
          <Grid item xs={12}>
            <Typography variant="subtitle2" gutterBottom>
              Animation Speed: {preferences?.interface_preferences.animation_speed || 'normal'}
            </Typography>
            <FormControl component="fieldset">
              <RadioGroup
                row
                value={preferences?.interface_preferences.animation_speed || 'normal'}
                onChange={(e) => updatePreferences({
                  interface_preferences: {
                    ...preferences?.interface_preferences,
                    animation_speed: e.target.value
                  }
                } as any)}
              >
                <FormControlLabel value="slow" control={<Radio />} label="Slow" />
                <FormControlLabel value="normal" control={<Radio />} label="Normal" />
                <FormControlLabel value="fast" control={<Radio />} label="Fast" />
                <FormControlLabel value="none" control={<Radio />} label="None" />
              </RadioGroup>
            </FormControl>
          </Grid>
        </Grid>
      </CardContent>
    </Card>
  );

  if (!user) {
    return (
      <Alert severity="info">
        Please log in to access personalized features.
      </Alert>
    );
  }

  return (
    <Box sx={{ p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Personalized Experience
      </Typography>
      
      <Typography variant="body1" paragraph>
        Your learning experience is tailored based on your preferences and usage patterns.
      </Typography>

      <Grid container spacing={3}>
        <Grid item xs={12} md={8}>
          {renderQuickSettings()}
          {renderLearningInsights()}
        </Grid>
        
        <Grid item xs={12} md={4}>
          <Card>
            <CardContent>
              <Typography variant="h6" gutterBottom>
                Your Progress
              </Typography>
              <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 2 }}>
                <Star color="primary" />
                <Typography variant="body2">
                  Personalization Level: Advanced
                </Typography>
              </Box>
              <Typography variant="body2" color="text.secondary" paragraph>
                The system has learned your preferences and is providing optimized content.
              </Typography>
              
              <Button
                variant="outlined"
                fullWidth
                onClick={() => setOnboardingOpen(true)}
                sx={{ mt: 2 }}
              >
                Review Preferences
              </Button>
              
              <Button
                variant="text"
                fullWidth
                onClick={() => setFeedbackDialog(true)}
                sx={{ mt: 1 }}
              >
                Provide Feedback
              </Button>
            </CardContent>
          </Card>
        </Grid>
      </Grid>

      {/* Onboarding Dialog */}
      <Dialog
        open={onboardingOpen}
        onClose={() => setOnboardingOpen(false)}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>
          <Box sx={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
            Personalize Your Experience
            <IconButton onClick={() => setOnboardingOpen(false)}>
              <Close />
            </IconButton>
          </Box>
        </DialogTitle>
        
        <DialogContent>
          <Stepper activeStep={activeStep} orientation="vertical">
            {[
              'Language Background',
              'Learning Style',
              'Interface Preferences',
              'Translation Settings'
            ].map((label, index) => (
              <Step key={label}>
                <StepLabel>{label}</StepLabel>
                <StepContent>
                  {renderOnboardingStep(index)}
                  <Box sx={{ mt: 2 }}>
                    <Button
                      variant="contained"
                      onClick={() => {
                        if (index === 3) {
                          completeOnboarding();
                        } else {
                          setActiveStep(index + 1);
                        }
                      }}
                      sx={{ mr: 1 }}
                    >
                      {index === 3 ? 'Complete' : 'Next'}
                    </Button>
                    {index > 0 && (
                      <Button onClick={() => setActiveStep(index - 1)}>
                        Back
                      </Button>
                    )}
                  </Box>
                </StepContent>
              </Step>
            ))}
          </Stepper>
        </DialogContent>
      </Dialog>

      {/* Feedback Dialog */}
      <Dialog
        open={feedbackDialog}
        onClose={() => setFeedbackDialog(false)}
        maxWidth="sm"
        fullWidth
      >
        <DialogTitle>Share Your Feedback</DialogTitle>
        <DialogContent>
          <Typography variant="body2" paragraph>
            Help us improve your multilingual learning experience.
          </Typography>
          <TextField
            fullWidth
            multiline
            rows={4}
            placeholder="Tell us about your experience with the multilingual features..."
            sx={{ mt: 2 }}
          />
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setFeedbackDialog(false)}>Cancel</Button>
          <Button variant="contained" onClick={() => setFeedbackDialog(false)}>
            Submit Feedback
          </Button>
        </DialogActions>
      </Dialog>
    </Box>
  );
};

export default PersonalizedExperience;