/**
 * Community Translation User Profile Component
 * Shows user's contribution history, achievements, and statistics
 */

import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Avatar,
  Chip,
  Grid,
  List,
  ListItem,
  ListItemText,
  ListItemIcon,
  Divider,
  LinearProgress,
  Tabs,
  Tab,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,

  Badge,
  Tooltip
} from '@mui/material';
import {
  Timeline,
  TimelineItem,
  TimelineSeparator,
  TimelineConnector,
  TimelineContent,
  TimelineDot,
} from '@mui/lab';
import {
  Star,
  Translate,
  TrendingUp,
  EmojiEvents,
  School,
  Timeline as TimelineIcon,
  Assignment,
  Comment,
  ThumbUp,
  Edit,
  CheckCircle,
  Cancel,
  Pending
} from '@mui/icons-material';
import { communityTranslationAPI, UserStats, UserContribution } from '../../services/communityTranslationAPI';
import { useAuth } from '../../auth/AuthContext';

interface UserProfileProps {
  userId?: number;
  compact?: boolean;
}

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
      id={`profile-tabpanel-${index}`}
      aria-labelledby={`profile-tab-${index}`}
      {...other}
    >
      {value === index && (
        <Box sx={{ p: 3 }}>
          {children}
        </Box>
      )}
    </div>
  );
}

export const UserProfile: React.FC<UserProfileProps> = ({
  userId,
  compact = false
}) => {
  const { user, token } = useAuth();
  const [activeTab, setActiveTab] = useState(0);
  const [loading, setLoading] = useState(false);
  const [userStats, setUserStats] = useState<UserStats | null>(null);
  const [contributions, setContributions] = useState<UserContribution[]>([]);
  const [reputationHistory, setReputationHistory] = useState<any[]>([]);
  
  const targetUserId = userId || user?.id;

  useEffect(() => {
    if (targetUserId && token) {
      loadUserData();
    }
  }, [targetUserId, token]);

  const loadUserData = async () => {
    if (!targetUserId || !token) return;
    
    setLoading(true);
    try {
      // Load user stats
      const stats = await communityTranslationAPI.getUserStats(targetUserId, token);
      setUserStats(stats);

      // Load contributions
      const contributionsResponse = await communityTranslationAPI.getUserContributions(
        targetUserId,
        undefined,
        50,
        0,
        token
      );
      setContributions(contributionsResponse.contributions);

      // Load reputation history
      const history = await communityTranslationAPI.getReputationHistory(targetUserId, 50, token);
      setReputationHistory(history);
    } catch (error) {
      console.error('Error loading user data:', error);
    } finally {
      setLoading(false);
    }
  };

  const getLevelColor = (level: string) => {
    const colors: Record<string, any> = {
      'Beginner': 'default',
      'Contributor': 'primary',
      'Translator': 'secondary',
      'Expert': 'success',
      'Master': 'warning'
    };
    return colors[level] || 'default';
  };

  const getStatusIcon = (status: string) => {
    switch (status) {
      case 'approved':
      case 'published':
        return <CheckCircle color="success" />;
      case 'rejected':
        return <Cancel color="error" />;
      case 'pending':
      case 'in_progress':
        return <Pending color="warning" />;
      default:
        return <Assignment />;
    }
  };

  const getContributionTypeIcon = (type: string) => {
    switch (type) {
      case 'translation':
        return <Translate />;
      case 'review':
        return <Star />;
      case 'correction':
        return <Edit />;
      case 'terminology':
        return <Comment />;
      default:
        return <Assignment />;
    }
  };

  const renderStatsOverview = () => {
    if (!userStats) return null;

    const nextLevelThresholds = {
      'Beginner': 50,
      'Contributor': 200,
      'Translator': 500,
      'Expert': 1000,
      'Master': Infinity
    };

    const currentThreshold = nextLevelThresholds[userStats.level as keyof typeof nextLevelThresholds];
    const progressToNext = currentThreshold === Infinity ? 100 : 
      (userStats.reputation / currentThreshold) * 100;

    return (
      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 3, mb: 3 }}>
            <Avatar sx={{ width: 80, height: 80, fontSize: '2rem' }}>
              {user?.username?.charAt(0).toUpperCase() || 'U'}
            </Avatar>
            <Box sx={{ flexGrow: 1 }}>
              <Typography variant="h5" gutterBottom>
                {user?.full_name || user?.username || 'User'}
              </Typography>
              <Box sx={{ display: 'flex', gap: 1, mb: 2 }}>
                <Chip 
                  label={userStats.level} 
                  color={getLevelColor(userStats.level)}
                  icon={<EmojiEvents />}
                />
                <Chip 
                  label={`${userStats.reputation} points`} 
                  variant="outlined"
                  icon={<Star />}
                />
              </Box>
              {currentThreshold !== Infinity && (
                <Box>
                  <Typography variant="body2" color="text.secondary" gutterBottom>
                    Progress to next level: {Math.round(progressToNext)}%
                  </Typography>
                  <LinearProgress 
                    variant="determinate" 
                    value={Math.min(progressToNext, 100)}
                    sx={{ height: 8, borderRadius: 4 }}
                  />
                </Box>
              )}
            </Box>
          </Box>

          <Grid container spacing={3}>
            <Grid item xs={6} sm={3}>
              <Box sx={{ textAlign: 'center' }}>
                <Typography variant="h4" color="primary.main">
                  {userStats.translation_count}
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  Translations
                </Typography>
              </Box>
            </Grid>
            <Grid item xs={6} sm={3}>
              <Box sx={{ textAlign: 'center' }}>
                <Typography variant="h4" color="success.main">
                  {userStats.approved_translations}
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  Approved
                </Typography>
              </Box>
            </Grid>
            <Grid item xs={6} sm={3}>
              <Box sx={{ textAlign: 'center' }}>
                <Typography variant="h4" color="secondary.main">
                  {userStats.approval_rate.toFixed(1)}%
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  Approval Rate
                </Typography>
              </Box>
            </Grid>
            <Grid item xs={6} sm={3}>
              <Box sx={{ textAlign: 'center' }}>
                <Typography variant="h4" color="warning.main">
                  {userStats.terminology_contributions}
                </Typography>
                <Typography variant="body2" color="text.secondary">
                  Terms Added
                </Typography>
              </Box>
            </Grid>
          </Grid>
        </CardContent>
      </Card>
    );
  };

  const renderPrivileges = () => {
    if (!userStats?.privileges?.length) return null;

    return (
      <Card sx={{ mb: 3 }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>
            Privileges & Permissions
          </Typography>
          <Box sx={{ display: 'flex', flexWrap: 'wrap', gap: 1 }}>
            {userStats.privileges.map((privilege) => (
              <Chip
                key={privilege}
                label={privilege.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase())}
                size="small"
                variant="outlined"
                color="primary"
              />
            ))}
          </Box>
        </CardContent>
      </Card>
    );
  };

  const renderContributionsTable = () => (
    <TableContainer component={Paper}>
      <Table>
        <TableHead>
          <TableRow>
            <TableCell>Type</TableCell>
            <TableCell>Content</TableCell>
            <TableCell>Language</TableCell>
            <TableCell>Status</TableCell>
            <TableCell>Rating</TableCell>
            <TableCell>Date</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {contributions.map((contribution) => (
            <TableRow key={`${contribution.type}-${contribution.id}`}>
              <TableCell>
                <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                  {getContributionTypeIcon(contribution.type)}
                  <Typography variant="body2" sx={{ textTransform: 'capitalize' }}>
                    {contribution.type}
                  </Typography>
                </Box>
              </TableCell>
              <TableCell>
                <Typography variant="body2" noWrap sx={{ maxWidth: 200 }}>
                  {contribution.title || contribution.content_reference || 'N/A'}
                </Typography>
              </TableCell>
              <TableCell>
                <Chip label={contribution.language || 'N/A'} size="small" />
              </TableCell>
              <TableCell>
                <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                  {getStatusIcon(contribution.status)}
                  <Typography variant="body2" sx={{ textTransform: 'capitalize' }}>
                    {contribution.status}
                  </Typography>
                </Box>
              </TableCell>
              <TableCell>
                {contribution.average_rating ? (
                  <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
                    <Star sx={{ fontSize: 16, color: 'warning.main' }} />
                    <Typography variant="body2">
                      {contribution.average_rating.toFixed(1)}
                    </Typography>
                    <Typography variant="caption" color="text.secondary">
                      ({contribution.reviews})
                    </Typography>
                  </Box>
                ) : (
                  <Typography variant="body2" color="text.secondary">
                    No reviews
                  </Typography>
                )}
              </TableCell>
              <TableCell>
                <Typography variant="body2">
                  {new Date(contribution.created_at).toLocaleDateString()}
                </Typography>
              </TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );

  const renderReputationTimeline = () => (
    <Timeline>
      {reputationHistory.map((entry, index) => (
        <TimelineItem key={index}>
          <TimelineSeparator>
            <TimelineDot 
              color={entry.points > 0 ? 'success' : 'error'}
              variant={entry.points > 0 ? 'filled' : 'outlined'}
            >
              {entry.points > 0 ? <TrendingUp /> : <Cancel />}
            </TimelineDot>
            {index < reputationHistory.length - 1 && <TimelineConnector />}
          </TimelineSeparator>
          <TimelineContent>
            <Typography variant="body2" sx={{ fontWeight: 'medium' }}>
              {entry.action.replace(/_/g, ' ').replace(/\b\w/g, (l: string) => l.toUpperCase())}
            </Typography>
            <Typography variant="body2" color="text.secondary">
              {entry.points > 0 ? '+' : ''}{entry.points} points
            </Typography>
            <Typography variant="caption" color="text.secondary">
              {new Date(entry.timestamp).toLocaleString()}
            </Typography>
            <Typography variant="caption" color="text.secondary" sx={{ display: 'block' }}>
              Total: {entry.total_reputation} points
            </Typography>
          </TimelineContent>
        </TimelineItem>
      ))}
    </Timeline>
  );

  if (!user || !token) {
    return (
      <Card>
        <CardContent>
          <Typography>Please log in to view profile.</Typography>
        </CardContent>
      </Card>
    );
  }

  if (compact) {
    return (
      <Card>
        <CardContent>
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 2, mb: 2 }}>
            <Avatar sx={{ width: 40, height: 40 }}>
              {user.username?.charAt(0).toUpperCase() || 'U'}
            </Avatar>
            <Box>
              <Typography variant="h6">
                {user.full_name || user.username}
              </Typography>
              {userStats && (
                <Chip 
                  label={userStats.level} 
                  color={getLevelColor(userStats.level)}
                  size="small"
                />
              )}
            </Box>
          </Box>
          {userStats && (
            <Grid container spacing={2}>
              <Grid item xs={6}>
                <Typography variant="body2" color="text.secondary">
                  Reputation: {userStats.reputation}
                </Typography>
              </Grid>
              <Grid item xs={6}>
                <Typography variant="body2" color="text.secondary">
                  Translations: {userStats.translation_count}
                </Typography>
              </Grid>
            </Grid>
          )}
        </CardContent>
      </Card>
    );
  }

  return (
    <Box>
      {loading && <LinearProgress sx={{ mb: 2 }} />}
      
      {renderStatsOverview()}
      {renderPrivileges()}

      <Card>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={activeTab} onChange={(_, newValue) => setActiveTab(newValue)}>
            <Tab label="Contributions" icon={<Assignment />} />
            <Tab label="Reputation History" icon={<TimelineIcon />} />
          </Tabs>
        </Box>

        <TabPanel value={activeTab} index={0}>
          <Typography variant="h6" gutterBottom>
            Recent Contributions
          </Typography>
          {contributions.length > 0 ? (
            renderContributionsTable()
          ) : (
            <Typography color="text.secondary">
              No contributions yet. Start contributing to build your reputation!
            </Typography>
          )}
        </TabPanel>

        <TabPanel value={activeTab} index={1}>
          <Typography variant="h6" gutterBottom>
            Reputation History
          </Typography>
          {reputationHistory.length > 0 ? (
            renderReputationTimeline()
          ) : (
            <Typography color="text.secondary">
              No reputation history available.
            </Typography>
          )}
        </TabPanel>
      </Card>
    </Box>
  );
};

export default UserProfile;