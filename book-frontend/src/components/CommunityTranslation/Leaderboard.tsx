/**
 * Community Translation Leaderboard Component
 * Displays top contributors and their achievements
 */

import React, { useState, useEffect } from 'react';
import {
  Box,
  Card,
  CardContent,
  Typography,
  Avatar,
  Chip,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableHead,
  TableRow,
  Paper,
  Tabs,
  Tab,
  Grid,
  LinearProgress,
  Tooltip,
  IconButton,
  Select,
  MenuItem,
  FormControl,
  InputLabel
} from '@mui/material';
import {
  EmojiEvents,
  Star,
  Translate,
  TrendingUp,
  Person,
  Refresh
} from '@mui/icons-material';
import { communityTranslationAPI, LeaderboardEntry } from '../../services/communityTranslationAPI';
import { useAuth } from '../../auth/AuthContext';

interface LeaderboardProps {
  compact?: boolean;
  showUserRank?: boolean;
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
      id={`leaderboard-tabpanel-${index}`}
      aria-labelledby={`leaderboard-tab-${index}`}
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

export const Leaderboard: React.FC<LeaderboardProps> = ({
  compact = false,
  showUserRank = true
}) => {
  const { user, token } = useAuth();
  const [activeTab, setActiveTab] = useState(0);
  const [loading, setLoading] = useState(false);
  const [leaderboardData, setLeaderboardData] = useState<LeaderboardEntry[]>([]);
  const [period, setPeriod] = useState('all_time');
  const [userRank, setUserRank] = useState<number | null>(null);
  const [qualityMetrics, setQualityMetrics] = useState<any>(null);

  const periods = [
    { value: 'week', label: 'This Week' },
    { value: 'month', label: 'This Month' },
    { value: 'year', label: 'This Year' },
    { value: 'all_time', label: 'All Time' }
  ];

  useEffect(() => {
    loadLeaderboard();
    loadQualityMetrics();
  }, [period]);

  const loadLeaderboard = async () => {
    setLoading(true);
    try {
      const response = await communityTranslationAPI.getLeaderboard(period, 50, token);
      setLeaderboardData(response.leaderboard);
      
      // Find user's rank if logged in
      if (user && showUserRank) {
        const userIndex = response.leaderboard.findIndex(entry => entry.user_id === user.id);
        setUserRank(userIndex >= 0 ? userIndex + 1 : null);
      }
    } catch (error) {
      console.error('Error loading leaderboard:', error);
    } finally {
      setLoading(false);
    }
  };

  const loadQualityMetrics = async () => {
    try {
      const metrics = await communityTranslationAPI.getQualityMetrics(undefined, undefined, token);
      setQualityMetrics(metrics);
    } catch (error) {
      console.error('Error loading quality metrics:', error);
    }
  };

  const getLevelColor = (level: string) => {
    const colors: Record<string, string> = {
      'Beginner': 'default',
      'Contributor': 'primary',
      'Translator': 'secondary',
      'Expert': 'success',
      'Master': 'warning'
    };
    return colors[level] || 'default';
  };

  const getRankIcon = (rank: number) => {
    if (rank === 1) return <EmojiEvents sx={{ color: '#FFD700' }} />;
    if (rank === 2) return <EmojiEvents sx={{ color: '#C0C0C0' }} />;
    if (rank === 3) return <EmojiEvents sx={{ color: '#CD7F32' }} />;
    return <Typography variant="body2" sx={{ fontWeight: 'bold' }}>{rank}</Typography>;
  };

  const renderTopContributors = () => (
    <Grid container spacing={2} sx={{ mb: 3 }}>
      {leaderboardData.slice(0, 3).map((contributor, index) => (
        <Grid item xs={12} sm={4} key={contributor.user_id}>
          <Card 
            sx={{ 
              textAlign: 'center',
              bgcolor: index === 0 ? 'primary.light' : 'background.paper',
              color: index === 0 ? 'primary.contrastText' : 'text.primary'
            }}
          >
            <CardContent>
              <Box sx={{ display: 'flex', justifyContent: 'center', mb: 2 }}>
                {getRankIcon(index + 1)}
              </Box>
              <Avatar sx={{ width: 60, height: 60, mx: 'auto', mb: 2 }}>
                {contributor.username.charAt(0).toUpperCase()}
              </Avatar>
              <Typography variant="h6" gutterBottom>
                {contributor.full_name || contributor.username}
              </Typography>
              <Chip 
                label={contributor.level} 
                color={getLevelColor(contributor.level) as any}
                size="small"
                sx={{ mb: 1 }}
              />
              <Typography variant="body2" sx={{ mb: 1 }}>
                {contributor.reputation} points
              </Typography>
              <Typography variant="body2">
                {contributor.translation_count} translations
              </Typography>
            </CardContent>
          </Card>
        </Grid>
      ))}
    </Grid>
  );

  const renderFullLeaderboard = () => (
    <TableContainer component={Paper}>
      <Table>
        <TableHead>
          <TableRow>
            <TableCell>Rank</TableCell>
            <TableCell>Contributor</TableCell>
            <TableCell>Level</TableCell>
            <TableCell align="right">Reputation</TableCell>
            <TableCell align="right">Translations</TableCell>
            <TableCell align="right">Period Contributions</TableCell>
          </TableRow>
        </TableHead>
        <TableBody>
          {leaderboardData.map((contributor, index) => (
            <TableRow 
              key={contributor.user_id}
              sx={{ 
                bgcolor: user && contributor.user_id === user.id ? 'action.selected' : 'inherit'
              }}
            >
              <TableCell>
                <Box sx={{ display: 'flex', alignItems: 'center' }}>
                  {getRankIcon(index + 1)}
                </Box>
              </TableCell>
              <TableCell>
                <Box sx={{ display: 'flex', alignItems: 'center', gap: 2 }}>
                  <Avatar sx={{ width: 32, height: 32 }}>
                    {contributor.username.charAt(0).toUpperCase()}
                  </Avatar>
                  <Box>
                    <Typography variant="body2" sx={{ fontWeight: 'medium' }}>
                      {contributor.full_name || contributor.username}
                    </Typography>
                    <Typography variant="caption" color="text.secondary">
                      @{contributor.username}
                    </Typography>
                  </Box>
                </Box>
              </TableCell>
              <TableCell>
                <Chip 
                  label={contributor.level} 
                  color={getLevelColor(contributor.level) as any}
                  size="small"
                />
              </TableCell>
              <TableCell align="right">
                <Typography variant="body2" sx={{ fontWeight: 'medium' }}>
                  {contributor.reputation.toLocaleString()}
                </Typography>
              </TableCell>
              <TableCell align="right">
                <Typography variant="body2">
                  {contributor.translation_count}
                </Typography>
              </TableCell>
              <TableCell align="right">
                <Typography variant="body2">
                  {contributor.period_contributions}
                </Typography>
              </TableCell>
            </TableRow>
          ))}
        </TableBody>
      </Table>
    </TableContainer>
  );

  const renderQualityMetrics = () => (
    <Grid container spacing={3}>
      <Grid item xs={12} sm={6} md={3}>
        <Card>
          <CardContent sx={{ textAlign: 'center' }}>
            <Star sx={{ fontSize: 40, color: 'primary.main', mb: 1 }} />
            <Typography variant="h4" gutterBottom>
              {qualityMetrics?.average_quality?.toFixed(1) || 'N/A'}
            </Typography>
            <Typography variant="body2" color="text.secondary">
              Average Quality Score
            </Typography>
          </CardContent>
        </Card>
      </Grid>
      <Grid item xs={12} sm={6} md={3}>
        <Card>
          <CardContent sx={{ textAlign: 'center' }}>
            <Translate sx={{ fontSize: 40, color: 'secondary.main', mb: 1 }} />
            <Typography variant="h4" gutterBottom>
              {qualityMetrics?.total_translations?.toLocaleString() || 'N/A'}
            </Typography>
            <Typography variant="body2" color="text.secondary">
              Total Translations
            </Typography>
          </CardContent>
        </Card>
      </Grid>
      <Grid item xs={12} sm={6} md={3}>
        <Card>
          <CardContent sx={{ textAlign: 'center' }}>
            <TrendingUp sx={{ fontSize: 40, color: 'success.main', mb: 1 }} />
            <Typography variant="h4" gutterBottom>
              {qualityMetrics?.approval_rate?.toFixed(1) || 'N/A'}%
            </Typography>
            <Typography variant="body2" color="text.secondary">
              Approval Rate
            </Typography>
          </CardContent>
        </Card>
      </Grid>
      <Grid item xs={12} sm={6} md={3}>
        <Card>
          <CardContent sx={{ textAlign: 'center' }}>
            <Person sx={{ fontSize: 40, color: 'warning.main', mb: 1 }} />
            <Typography variant="h4" gutterBottom>
              {qualityMetrics?.active_contributors || 'N/A'}
            </Typography>
            <Typography variant="body2" color="text.secondary">
              Active Contributors
            </Typography>
          </CardContent>
        </Card>
      </Grid>
    </Grid>
  );

  const renderUserRankCard = () => {
    if (!user || !userRank) return null;

    return (
      <Card sx={{ mb: 3, bgcolor: 'primary.light', color: 'primary.contrastText' }}>
        <CardContent>
          <Typography variant="h6" gutterBottom>
            Your Ranking
          </Typography>
          <Grid container spacing={2} alignItems="center">
            <Grid item>
              <Typography variant="h4">
                #{userRank}
              </Typography>
            </Grid>
            <Grid item xs>
              <Typography variant="body2">
                You're ranked #{userRank} out of {leaderboardData.length} contributors
              </Typography>
              <LinearProgress 
                variant="determinate" 
                value={(leaderboardData.length - userRank + 1) / leaderboardData.length * 100}
                sx={{ mt: 1, bgcolor: 'rgba(255,255,255,0.3)' }}
              />
            </Grid>
          </Grid>
        </CardContent>
      </Card>
    );
  };

  if (compact) {
    return (
      <Card>
        <CardContent>
          <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
            <Typography variant="h6">Top Contributors</Typography>
            <Tooltip title="Refresh">
              <IconButton onClick={loadLeaderboard} disabled={loading}>
                <Refresh />
              </IconButton>
            </Tooltip>
          </Box>
          {loading && <LinearProgress sx={{ mb: 2 }} />}
          {leaderboardData.slice(0, 5).map((contributor, index) => (
            <Box 
              key={contributor.user_id}
              sx={{ 
                display: 'flex', 
                alignItems: 'center', 
                gap: 2, 
                py: 1,
                borderBottom: index < 4 ? '1px solid' : 'none',
                borderColor: 'divider'
              }}
            >
              <Typography variant="body2" sx={{ minWidth: 20 }}>
                {index + 1}
              </Typography>
              <Avatar sx={{ width: 24, height: 24 }}>
                {contributor.username.charAt(0).toUpperCase()}
              </Avatar>
              <Box sx={{ flexGrow: 1 }}>
                <Typography variant="body2">
                  {contributor.full_name || contributor.username}
                </Typography>
              </Box>
              <Typography variant="body2" color="text.secondary">
                {contributor.reputation}
              </Typography>
            </Box>
          ))}
        </CardContent>
      </Card>
    );
  }

  return (
    <Box>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Typography variant="h4" gutterBottom>
          Community Leaderboard
        </Typography>
        <Box sx={{ display: 'flex', gap: 2, alignItems: 'center' }}>
          <FormControl size="small" sx={{ minWidth: 120 }}>
            <InputLabel>Period</InputLabel>
            <Select
              value={period}
              label="Period"
              onChange={(e) => setPeriod(e.target.value)}
            >
              {periods.map((p) => (
                <MenuItem key={p.value} value={p.value}>
                  {p.label}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
          <Tooltip title="Refresh">
            <IconButton onClick={loadLeaderboard} disabled={loading}>
              <Refresh />
            </IconButton>
          </Tooltip>
        </Box>
      </Box>

      {loading && <LinearProgress sx={{ mb: 3 }} />}

      <Box sx={{ borderBottom: 1, borderColor: 'divider', mb: 3 }}>
        <Tabs value={activeTab} onChange={(_, newValue) => setActiveTab(newValue)}>
          <Tab label="Rankings" />
          <Tab label="Statistics" />
        </Tabs>
      </Box>

      <TabPanel value={activeTab} index={0}>
        {showUserRank && renderUserRankCard()}
        {renderTopContributors()}
        {renderFullLeaderboard()}
      </TabPanel>

      <TabPanel value={activeTab} index={1}>
        {renderQualityMetrics()}
      </TabPanel>
    </Box>
  );
};

export default Leaderboard;