/**
 * Community Translation Demo Page
 * Showcases all community translation features
 */

import React, { useState } from 'react';
import {
  Box,
  Container,
  Typography,
  Grid,
  Card,
  CardContent,
  Tabs,
  Tab,
  Button,
  Alert,
  Divider,
  Paper
} from '@mui/material';
import {
  Translate,
  EmojiEvents,
  Person,
  Assignment,
  Info
} from '@mui/icons-material';
import {
  ContributionInterface,
  Leaderboard,
  UserProfile
} from '../components/CommunityTranslation';
import { useAuth } from '../auth/AuthContext';

interface TabPanelProps {
  children?: React.ReactNode;
  index: number;
  value: number;
}

function TabPanel(props: TabPanelProps) {
  const { children, value, index, ...other } = props;

  return (
    <div
      role="tabpanel"
      hidden={value !== index}
      id={`demo-tabpanel-${index}`}
      aria-labelledby={`demo-tab-${index}`}
      {...other}
    >
      {value === index && (
        <Box sx={{ py: 3 }}>
          {children}
        </Box>
      )}
    </div>
  );
}

export const CommunityTranslationDemo: React.FC = () => {
  const { user } = useAuth();
  const [activeTab, setActiveTab] = useState(0);
  const [demoContent] = useState({
    contentType: 'chapter',
    contentId: 'demo-chapter-1',
    sourceLanguage: 'en',
    targetLanguage: 'ur',
    originalTitle: 'Introduction to Machine Learning',
    originalContent: `Machine Learning is a subset of artificial intelligence (AI) that provides systems the ability to automatically learn and improve from experience without being explicitly programmed. Machine learning focuses on the development of computer programs that can access data and use it to learn for themselves.

The process of learning begins with observations or data, such as examples, direct experience, or instruction, in order to look for patterns in data and make better decisions in the future based on the examples that we provide. The primary aim is to allow the computers to learn automatically without human intervention or assistance and adjust actions accordingly.`
  });

  const handleContributionSubmitted = () => {
    console.log('Contribution submitted successfully');
  };

  const renderIntroduction = () => (
    <Paper sx={{ p: 3, mb: 3 }}>
      <Typography variant="h4" gutterBottom>
        Community Translation System
      </Typography>
      <Typography variant="body1" paragraph>
        Welcome to our community-driven translation system! This platform enables users to contribute 
        translations, review existing translations, and help improve the quality of multilingual content.
      </Typography>
      
      <Grid container spacing={3} sx={{ mt: 2 }}>
        <Grid item xs={12} md={3}>
          <Card sx={{ textAlign: 'center', height: '100%' }}>
            <CardContent>
              <Translate sx={{ fontSize: 40, color: 'primary.main', mb: 2 }} />
              <Typography variant="h6" gutterBottom>
                Contribute Translations
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Submit your own translations and help make content accessible in multiple languages.
              </Typography>
            </CardContent>
          </Card>
        </Grid>
        
        <Grid item xs={12} md={3}>
          <Card sx={{ textAlign: 'center', height: '100%' }}>
            <CardContent>
              <Assignment sx={{ fontSize: 40, color: 'secondary.main', mb: 2 }} />
              <Typography variant="h6" gutterBottom>
                Review & Improve
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Review translations from other contributors and suggest improvements.
              </Typography>
            </CardContent>
          </Card>
        </Grid>
        
        <Grid item xs={12} md={3}>
          <Card sx={{ textAlign: 'center', height: '100%' }}>
            <CardContent>
              <EmojiEvents sx={{ fontSize: 40, color: 'warning.main', mb: 2 }} />
              <Typography variant="h6" gutterBottom>
                Earn Recognition
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Build your reputation and climb the leaderboard through quality contributions.
              </Typography>
            </CardContent>
          </Card>
        </Grid>
        
        <Grid item xs={12} md={3}>
          <Card sx={{ textAlign: 'center', height: '100%' }}>
            <CardContent>
              <Person sx={{ fontSize: 40, color: 'success.main', mb: 2 }} />
              <Typography variant="h6" gutterBottom>
                Track Progress
              </Typography>
              <Typography variant="body2" color="text.secondary">
                Monitor your contributions and see your impact on the community.
              </Typography>
            </CardContent>
          </Card>
        </Grid>
      </Grid>
    </Paper>
  );

  const renderDemoContent = () => (
    <Paper sx={{ p: 3, mb: 3 }}>
      <Typography variant="h6" gutterBottom>
        Demo Content - English Original
      </Typography>
      <Typography variant="h5" gutterBottom color="primary">
        {demoContent.originalTitle}
      </Typography>
      <Typography variant="body1" sx={{ whiteSpace: 'pre-wrap', lineHeight: 1.6 }}>
        {demoContent.originalContent}
      </Typography>
      <Divider sx={{ my: 2 }} />
      <Typography variant="body2" color="text.secondary">
        This is sample content that you can practice translating. In a real scenario, 
        this would be actual book content that needs translation.
      </Typography>
    </Paper>
  );

  return (
    <Container maxWidth="lg" sx={{ py: 4 }}>
      {renderIntroduction()}
      
      {!user && (
        <Alert severity="info" sx={{ mb: 3 }}>
          <Typography variant="body2">
            Please log in to access all community translation features. Some features like 
            submitting translations and viewing detailed statistics require authentication.
          </Typography>
        </Alert>
      )}

      <Box sx={{ borderBottom: 1, borderColor: 'divider', mb: 3 }}>
        <Tabs value={activeTab} onChange={(_, newValue) => setActiveTab(newValue)}>
          <Tab 
            label="Contribute" 
            icon={<Translate />} 
            id="demo-tab-0"
            aria-controls="demo-tabpanel-0"
          />
          <Tab 
            label="Leaderboard" 
            icon={<EmojiEvents />} 
            id="demo-tab-1"
            aria-controls="demo-tabpanel-1"
          />
          <Tab 
            label="Profile" 
            icon={<Person />} 
            id="demo-tab-2"
            aria-controls="demo-tabpanel-2"
          />
          <Tab 
            label="Demo Info" 
            icon={<Info />} 
            id="demo-tab-3"
            aria-controls="demo-tabpanel-3"
          />
        </Tabs>
      </Box>

      <TabPanel value={activeTab} index={0}>
        <Typography variant="h5" gutterBottom>
          Translation Contribution Interface
        </Typography>
        <Typography variant="body1" paragraph color="text.secondary">
          Use this interface to submit translations, reviews, corrections, and terminology. 
          The system supports multiple types of contributions with a reputation-based reward system.
        </Typography>
        
        {renderDemoContent()}
        
        <ContributionInterface
          contentType={demoContent.contentType}
          contentId={demoContent.contentId}
          sourceLanguage={demoContent.sourceLanguage}
          targetLanguage={demoContent.targetLanguage}
          originalTitle={demoContent.originalTitle}
          originalContent={demoContent.originalContent}
          onContributionSubmitted={handleContributionSubmitted}
        />
      </TabPanel>

      <TabPanel value={activeTab} index={1}>
        <Typography variant="h5" gutterBottom>
          Community Leaderboard
        </Typography>
        <Typography variant="body1" paragraph color="text.secondary">
          See top contributors and their achievements. The leaderboard shows reputation points, 
          contribution counts, and user levels across different time periods.
        </Typography>
        
        <Leaderboard showUserRank={!!user} />
      </TabPanel>

      <TabPanel value={activeTab} index={2}>
        <Typography variant="h5" gutterBottom>
          User Profile & Statistics
        </Typography>
        <Typography variant="body1" paragraph color="text.secondary">
          View detailed user statistics, contribution history, and reputation timeline. 
          Track your progress and see how your contributions impact the community.
        </Typography>
        
        {user ? (
          <UserProfile />
        ) : (
          <Alert severity="info">
            Please log in to view your profile and contribution statistics.
          </Alert>
        )}
      </TabPanel>

      <TabPanel value={activeTab} index={3}>
        <Typography variant="h5" gutterBottom>
          Demo Information
        </Typography>
        
        <Grid container spacing={3}>
          <Grid item xs={12} md={6}>
            <Card>
              <CardContent>
                <Typography variant="h6" gutterBottom>
                  Features Demonstrated
                </Typography>
                <Typography component="div" variant="body2">
                  <ul>
                    <li>Translation submission with notes</li>
                    <li>Review system with ratings and feedback</li>
                    <li>Correction suggestions with explanations</li>
                    <li>Terminology contributions</li>
                    <li>Voting system for community validation</li>
                    <li>Reputation and leveling system</li>
                    <li>Leaderboard with multiple time periods</li>
                    <li>User profile with contribution history</li>
                    <li>Quality metrics and statistics</li>
                  </ul>
                </Typography>
              </CardContent>
            </Card>
          </Grid>
          
          <Grid item xs={12} md={6}>
            <Card>
              <CardContent>
                <Typography variant="h6" gutterBottom>
                  User Levels & Privileges
                </Typography>
                <Typography component="div" variant="body2">
                  <ul>
                    <li><strong>Beginner (0-49 points):</strong> Submit suggestions, vote</li>
                    <li><strong>Contributor (50-199 points):</strong> Submit translations, review</li>
                    <li><strong>Translator (200-499 points):</strong> Approve suggestions, edit terminology</li>
                    <li><strong>Expert (500-999 points):</strong> Moderate content, mentor users</li>
                    <li><strong>Master (1000+ points):</strong> Admin privileges, system configuration</li>
                  </ul>
                </Typography>
              </CardContent>
            </Card>
          </Grid>
          
          <Grid item xs={12}>
            <Card>
              <CardContent>
                <Typography variant="h6" gutterBottom>
                  Reputation System
                </Typography>
                <Typography variant="body2" paragraph>
                  Users earn reputation points through various activities:
                </Typography>
                <Grid container spacing={2}>
                  <Grid item xs={12} sm={6}>
                    <Typography variant="body2">
                      • Translation submitted: +5 points<br/>
                      • Translation approved: +20 points<br/>
                      • Review submitted: +3 points<br/>
                      • Review marked helpful: +10 points
                    </Typography>
                  </Grid>
                  <Grid item xs={12} sm={6}>
                    <Typography variant="body2">
                      • Terminology added: +15 points<br/>
                      • Correction accepted: +8 points<br/>
                      • Translation rejected: -5 points
                    </Typography>
                  </Grid>
                </Grid>
              </CardContent>
            </Card>
          </Grid>
        </Grid>
      </TabPanel>
    </Container>
  );
};

export default CommunityTranslationDemo;