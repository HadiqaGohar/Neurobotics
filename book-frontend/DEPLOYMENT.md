# ChatBot Deployment Guide

## Issue Fixed: ChatBot not showing on Vercel deployment

### Problem
ChatBot component was not visible after Vercel deployment even though it worked locally.

### Root Causes & Solutions

1. **API Configuration Issue**
   - **Problem**: Production API URL was set to placeholder `https://your-backend-url.com`
   - **Solution**: Modified API to use demo/mock mode when no backend is available

2. **CSS Specificity Issues**
   - **Problem**: Docusaurus styles might override ChatBot styles
   - **Solution**: Added `!important` declarations and improved CSS specificity

3. **SSR Compatibility**
   - **Problem**: ChatBot might not render properly during server-side rendering
   - **Solution**: Used `BrowserOnly` wrapper and proper error handling

### Changes Made

#### 1. API Layer (`src/components/ChatBot/api.ts`)
- Added fallback mock responses when API is unavailable
- Improved error handling with graceful degradation
- Demo mode for production without backend

#### 2. CSS Improvements (`src/components/ChatBot/styles.css` & `src/css/custom.css`)
- Added `!important` declarations for critical styles
- Improved z-index management
- Added pointer-events handling
- Dark theme compatibility

#### 3. Component Integration
- Ensured ChatBot is properly imported in `Root.tsx`
- Added debug component for troubleshooting
- Improved error boundaries

#### 4. Build Configuration
- Added `vercel.json` for proper deployment
- Environment variable handling
- Production build optimization

### Deployment Steps

1. **Local Testing**
   ```bash
   npm run build
   npm run serve
   ```

2. **Vercel Deployment**
   - Push to GitHub repository
   - Connect to Vercel
   - Deploy automatically

3. **Environment Variables** (Optional)
   - `REACT_APP_API_URL`: Your backend API URL
   - Leave empty for demo mode

### Features Available in Demo Mode

- ✅ Floating chat icon
- ✅ Chat window with responsive design
- ✅ Mock AI responses
- ✅ Voice input UI (mock)
- ✅ File upload UI (mock)
- ✅ Copy to clipboard
- ✅ Window resizing
- ✅ Dark/light theme support

### Verification

After deployment, check:
1. Floating chat icon appears in bottom-right corner
2. Click icon opens chat window
3. Send a message receives mock response
4. All UI features work properly
5. Responsive design on mobile

### Debug Mode

Add `?debug=true` to URL to see debug information in top-left corner.

### Next Steps

To connect real backend:
1. Set `REACT_APP_API_URL` environment variable
2. Ensure backend API matches the expected endpoints
3. Update CORS settings on backend
4. Test API connectivity

## Support

If ChatBot still doesn't appear:
1. Check browser console for errors
2. Verify build completed successfully
3. Test in incognito mode
4. Check if ad blockers are interfering