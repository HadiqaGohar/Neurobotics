/**
 * Community Translation Contribution Interface
 * Allows users to submit, review, and vote on translations
 */

import React, { useState, useEffect } from 'react';
import { 
  Box, 
  Card, 
  CardContent, 
  Typography, 
  TextField, 
  Button, 
  Tabs, 
  Tab, 
  Rating, 
  Chip, 
  Alert, 
  Dialog, 
  DialogTitle, 
  DialogContent, 
  DialogActions,
  Grid,
  IconButton,
  Tooltip,
  LinearProgress
} from '@mui/material';
import { 
  Translate, 
  ThumbUp, 
  ThumbDown, 
  Edit, 
  Visibility, 
  Star, 
  Comment,
  Send,
  Close
} from '@mui/icons-material';
import { useAuth } from '../../auth/AuthContext';
import { communityTranslationAPI } from '../../services/communityTranslationAPI';

interface ContributionInterfaceProps {
  contentType: string;
  contentId: string;
  sourceLanguage: string;
  targetLanguage: string;
  originalTitle?: string;
  originalContent?: string;
  onContributionSubmitted?: () => void;
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
      id={`contribution-tabpanel-${index}`}
      aria-labelledby={`contribution-tab-${index}`}
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

export const ContributionInterface: React.FC<ContributionInterfaceProps> = ({
  contentType,
  contentId,
  sourceLanguage,
  targetLanguage,
  originalTitle,
  originalContent,
  onContributionSubmitted
}) => {
  const { user, token } = useAuth();
  const [activeTab, setActiveTab] = useState(0);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);
  
  // Translation submission state
  const [translationTitle, setTranslationTitle] = useState('');
  const [translationContent, setTranslationContent] = useState('');
  const [translationNotes, setTranslationNotes] = useState('');
  
  // Review state
  const [reviewRating, setReviewRating] = useState<number | null>(null);
  const [reviewFeedback, setReviewFeedback] = useState('');
  const [suggestedImprovements, setSuggestedImprovements] = useState('');
  
  // Correction state
  const [corrections, setCorrections] = useState<Record<string, string>>({});
  const [correctionExplanation, setCorrectionExplanation] = useState('');
  
  // Terminology state
  const [term, setTerm] = useState('');
  const [termTranslation, setTermTranslation] = useState('');
  const [termDefinition, setTermDefinition] = useState('');
  const [termContext, setTermContext] = useState('');
  const [termDomain, setTermDomain] = useState('technical');
  
  // Existing contributions
  const [existingContributions, setExistingContributions] = useState<any[]>([]);
  const [userStats, setUserStats] = useState<any>(null);
  
  // Dialog state
  const [previewDialog, setPreviewDialog] = useState(false);
  const [previewContent, setPreviewContent] = useState<any>(null);

  useEffect(() => {
    loadExistingContributions();
    loadUserStats();
  }, [contentType, contentId, targetLanguage]);

  const loadExistingContributions = async () => {
    try {
      const response = await communityTranslationAPI.getContentContributions(
        contentType,
        contentId,
        targetLanguage,
        token!
      );
      setExistingContributions(response.contributions || []);
    } catch (error) {
      console.error('Error loading existing contributions:', error);
    }
  };

  const loadUserStats = async () => {
    if (!user || !token) return;
    
    try {
      const stats = await communityTranslationAPI.getUserStats(user.id, token);
      setUserStats(stats);
    } catch (error) {
      console.error('Error loading user stats:', error);
    }
  };

  const handleSubmitTranslation = async () => {
    if (!translationTitle.trim() && !translationContent.trim()) {
      setError('Please provide either a title or content translation');
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await communityTranslationAPI.submitTranslation(
        {
          content_type: contentType,
          content_id: contentId,
          language_code: targetLanguage,
          title: translationTitle.trim() || undefined,
          content: translationContent.trim() || undefined,
          notes: translationNotes.trim() || undefined
        },
        token!
      );

      if (response.success) {
        setSuccess('Translation submitted successfully!');
        setTranslationTitle('');
        setTranslationContent('');
        setTranslationNotes('');
        loadExistingContributions();
        onContributionSubmitted?.();
      } else {
        setError(response.error || 'Failed to submit translation');
      }
    } catch (error) {
      setError('Error submitting translation');
      console.error('Translation submission error:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleSubmitReview = async (translationId: number) => {
    if (!reviewRating || !reviewFeedback.trim()) {
      setError('Please provide both rating and feedback');
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const improvements = suggestedImprovements.trim() 
        ? { general: suggestedImprovements.trim() }
        : undefined;

      const response = await communityTranslationAPI.submitReview(
        translationId,
        {
          rating: reviewRating,
          feedback: reviewFeedback.trim(),
          suggested_improvements: improvements
        },
        token!
      );

      if (response.success) {
        setSuccess('Review submitted successfully!');
        setReviewRating(null);
        setReviewFeedback('');
        setSuggestedImprovements('');
        loadExistingContributions();
      } else {
        setError(response.error || 'Failed to submit review');
      }
    } catch (error) {
      setError('Error submitting review');
      console.error('Review submission error:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleSubmitCorrection = async (translationId: number) => {
    if (Object.keys(corrections).length === 0 || !correctionExplanation.trim()) {
      setError('Please provide corrections and explanation');
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await communityTranslationAPI.submitCorrection(
        translationId,
        {
          corrections,
          explanation: correctionExplanation.trim()
        },
        token!
      );

      if (response.success) {
        setSuccess('Correction submitted successfully!');
        setCorrections({});
        setCorrectionExplanation('');
        loadExistingContributions();
      } else {
        setError(response.error || 'Failed to submit correction');
      }
    } catch (error) {
      setError('Error submitting correction');
      console.error('Correction submission error:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleSubmitTerminology = async () => {
    if (!term.trim() || !termTranslation.trim()) {
      setError('Please provide both term and translation');
      return;
    }

    setLoading(true);
    setError(null);

    try {
      const response = await communityTranslationAPI.submitTerminology(
        {
          term: term.trim(),
          translation: termTranslation.trim(),
          source_language: sourceLanguage,
          target_language: targetLanguage,
          definition: termDefinition.trim() || undefined,
          context: termContext.trim() || undefined,
          domain: termDomain
        },
        token!
      );

      if (response.success) {
        setSuccess('Terminology submitted successfully!');
        setTerm('');
        setTermTranslation('');
        setTermDefinition('');
        setTermContext('');
      } else {
        setError(response.error || 'Failed to submit terminology');
      }
    } catch (error) {
      setError('Error submitting terminology');
      console.error('Terminology submission error:', error);
    } finally {
      setLoading(false);
    }
  };

  const handleVote = async (contributionId: number, voteType: 'upvote' | 'downvote') => {
    try {
      const response = await communityTranslationAPI.voteOnContribution(
        contributionId,
        'translation',
        voteType,
        token!
      );

      if (response.success) {
        loadExistingContributions();
      } else {
        setError(response.error || 'Failed to vote');
      }
    } catch (error) {
      setError('Error voting on contribution');
      console.error('Voting error:', error);
    }
  };

  const handlePreview = (contribution: any) => {
    setPreviewContent(contribution);
    setPreviewDialog(true);
  };

  const renderUserLevel = () => {
    if (!userStats) return null;

    return (
      <Box sx={{ mb: 2, p: 2, bgcolor: 'background.paper', borderRadius: 1 }}>
        <Typography variant="h6" gutterBottom>
          Your Contributor Profile
        </Typography>
        <Grid container spacing={2}>
          <Grid item xs={12} sm={6}>
            <Typography variant="body2" color="text.secondary">
              Level: <Chip label={userStats.level} color="primary" size="small" />
            </Typography>
            <Typography variant="body2" color="text.secondary">
              Reputation: {userStats.reputation} points
            </Typography>
          </Grid>
          <Grid item xs={12} sm={6}>
            <Typography variant="body2" color="text.secondary">
              Translations: {userStats.translation_count}
            </Typography>
            <Typography variant="body2" color="text.secondary">
              Approval Rate: {userStats.approval_rate?.toFixed(1)}%
            </Typography>
          </Grid>
        </Grid>
      </Box>
    );
  };

  const renderContributionCard = (contribution: any) => (
    <Card key={contribution.id} sx={{ mb: 2 }}>
      <CardContent>
        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'flex-start', mb: 2 }}>
          <Box>
            <Typography variant="h6" gutterBottom>
              {contribution.title || 'Content Translation'}
            </Typography>
            <Typography variant="body2" color="text.secondary">
              Status: <Chip label={contribution.status} size="small" />
            </Typography>
          </Box>
          <Box sx={{ display: 'flex', gap: 1 }}>
            <Tooltip title="Preview">
              <IconButton onClick={() => handlePreview(contribution)}>
                <Visibility />
              </IconButton>
            </Tooltip>
            <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
              <IconButton 
                onClick={() => handleVote(contribution.id, 'upvote')}
                color="primary"
              >
                <ThumbUp />
              </IconButton>
              <Typography variant="body2">{contribution.votes?.upvotes || 0}</Typography>
              <IconButton 
                onClick={() => handleVote(contribution.id, 'downvote')}
                color="secondary"
              >
                <ThumbDown />
              </IconButton>
              <Typography variant="body2">{contribution.votes?.downvotes || 0}</Typography>
            </Box>
          </Box>
        </Box>
        
        {contribution.average_rating && (
          <Box sx={{ display: 'flex', alignItems: 'center', gap: 1, mb: 1 }}>
            <Rating value={contribution.average_rating} readOnly size="small" />
            <Typography variant="body2" color="text.secondary">
              ({contribution.review_count} reviews)
            </Typography>
          </Box>
        )}
        
        <Typography variant="body2" color="text.secondary">
          Submitted: {new Date(contribution.created_at).toLocaleDateString()}
        </Typography>
      </CardContent>
    </Card>
  );

  if (!user) {
    return (
      <Alert severity="info">
        Please log in to contribute to translations.
      </Alert>
    );
  }

  return (
    <Box>
      {renderUserLevel()}
      
      {error && (
        <Alert severity="error" sx={{ mb: 2 }} onClose={() => setError(null)}>
          {error}
        </Alert>
      )}
      
      {success && (
        <Alert severity="success" sx={{ mb: 2 }} onClose={() => setSuccess(null)}>
          {success}
        </Alert>
      )}

      <Card>
        <Box sx={{ borderBottom: 1, borderColor: 'divider' }}>
          <Tabs value={activeTab} onChange={(_, newValue) => setActiveTab(newValue)}>
            <Tab label="Submit Translation" icon={<Translate />} />
            <Tab label="Review Translations" icon={<Star />} />
            <Tab label="Submit Corrections" icon={<Edit />} />
            <Tab label="Add Terminology" icon={<Comment />} />
          </Tabs>
        </Box>

        <TabPanel value={activeTab} index={0}>
          <Typography variant="h6" gutterBottom>
            Submit New Translation
          </Typography>
          <Typography variant="body2" color="text.secondary" sx={{ mb: 3 }}>
            Help improve translations by submitting your own version.
          </Typography>
          
          <Grid container spacing={2}>
            <Grid item xs={12}>
              <TextField
                fullWidth
                label="Translation Title"
                value={translationTitle}
                onChange={(e) => setTranslationTitle(e.target.value)}
                placeholder={originalTitle}
                helperText="Translate the title into the target language"
              />
            </Grid>
            <Grid item xs={12}>
              <TextField
                fullWidth
                multiline
                rows={8}
                label="Translation Content"
                value={translationContent}
                onChange={(e) => setTranslationContent(e.target.value)}
                placeholder="Enter your translation here..."
                helperText="Provide the translated content"
              />
            </Grid>
            <Grid item xs={12}>
              <TextField
                fullWidth
                multiline
                rows={3}
                label="Notes (Optional)"
                value={translationNotes}
                onChange={(e) => setTranslationNotes(e.target.value)}
                placeholder="Any notes about your translation approach..."
                helperText="Optional notes for reviewers"
              />
            </Grid>
            <Grid item xs={12}>
              <Button
                variant="contained"
                onClick={handleSubmitTranslation}
                disabled={loading}
                startIcon={<Send />}
              >
                {loading ? 'Submitting...' : 'Submit Translation'}
              </Button>
            </Grid>
          </Grid>
        </TabPanel>

        <TabPanel value={activeTab} index={1}>
          <Typography variant="h6" gutterBottom>
            Review Existing Translations
          </Typography>
          <Typography variant="body2" color="text.secondary" sx={{ mb: 3 }}>
            Help improve translation quality by reviewing submissions.
          </Typography>
          
          {existingContributions.length > 0 ? (
            existingContributions.map(renderContributionCard)
          ) : (
            <Alert severity="info">
              No translations available for review.
            </Alert>
          )}
        </TabPanel>

        <TabPanel value={activeTab} index={2}>
          <Typography variant="h6" gutterBottom>
            Submit Corrections
          </Typography>
          <Typography variant="body2" color="text.secondary" sx={{ mb: 3 }}>
            Suggest corrections to improve existing translations.
          </Typography>
          
          <Grid container spacing={2}>
            <Grid item xs={12}>
              <TextField
                fullWidth
                multiline
                rows={4}
                label="Corrections"
                value={corrections.general || ''}
                onChange={(e) => setCorrections({ ...corrections, general: e.target.value })}
                placeholder="Describe the corrections needed..."
                helperText="Specify what needs to be corrected"
              />
            </Grid>
            <Grid item xs={12}>
              <TextField
                fullWidth
                multiline
                rows={3}
                label="Explanation"
                value={correctionExplanation}
                onChange={(e) => setCorrectionExplanation(e.target.value)}
                placeholder="Explain why these corrections are needed..."
                helperText="Provide reasoning for your corrections"
              />
            </Grid>
          </Grid>
        </TabPanel>

        <TabPanel value={activeTab} index={3}>
          <Typography variant="h6" gutterBottom>
            Add Terminology
          </Typography>
          <Typography variant="body2" color="text.secondary" sx={{ mb: 3 }}>
            Contribute to the terminology glossary to improve translation consistency.
          </Typography>
          
          <Grid container spacing={2}>
            <Grid item xs={12} sm={6}>
              <TextField
                fullWidth
                label="Original Term"
                value={term}
                onChange={(e) => setTerm(e.target.value)}
                placeholder="Enter the original term..."
              />
            </Grid>
            <Grid item xs={12} sm={6}>
              <TextField
                fullWidth
                label="Translation"
                value={termTranslation}
                onChange={(e) => setTermTranslation(e.target.value)}
                placeholder="Enter the translation..."
              />
            </Grid>
            <Grid item xs={12}>
              <TextField
                fullWidth
                multiline
                rows={2}
                label="Definition (Optional)"
                value={termDefinition}
                onChange={(e) => setTermDefinition(e.target.value)}
                placeholder="Define the term..."
              />
            </Grid>
            <Grid item xs={12}>
              <TextField
                fullWidth
                multiline
                rows={2}
                label="Context (Optional)"
                value={termContext}
                onChange={(e) => setTermContext(e.target.value)}
                placeholder="Provide context for usage..."
              />
            </Grid>
            <Grid item xs={12}>
              <Button
                variant="contained"
                onClick={handleSubmitTerminology}
                disabled={loading}
                startIcon={<Send />}
              >
                {loading ? 'Submitting...' : 'Add Terminology'}
              </Button>
            </Grid>
          </Grid>
        </TabPanel>
      </Card>

      {/* Preview Dialog */}
      <Dialog 
        open={previewDialog} 
        onClose={() => setPreviewDialog(false)}
        maxWidth="md"
        fullWidth
      >
        <DialogTitle>
          Translation Preview
          <IconButton
            onClick={() => setPreviewDialog(false)}
            sx={{ position: 'absolute', right: 8, top: 8 }}
          >
            <Close />
          </IconButton>
        </DialogTitle>
        <DialogContent>
          {previewContent && (
            <Box>
              <Typography variant="h6" gutterBottom>
                {previewContent.title}
              </Typography>
              <Typography variant="body1" sx={{ whiteSpace: 'pre-wrap' }}>
                {previewContent.content}
              </Typography>
            </Box>
          )}
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setPreviewDialog(false)}>Close</Button>
        </DialogActions>
      </Dialog>

      {loading && <LinearProgress sx={{ mt: 2 }} />}
    </Box>
  );
};

export default ContributionInterface;