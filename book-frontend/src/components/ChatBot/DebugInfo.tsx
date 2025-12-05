import React, { useEffect, useState } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const DebugInfo: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();
  const [debugInfo, setDebugInfo] = useState<any>({});
  
  // Get environment info safely
  const isDevelopment = typeof window !== 'undefined' && window.location.hostname === 'localhost';

  useEffect(() => {
    if (typeof window !== 'undefined') {
      setDebugInfo({
        userAgent: navigator.userAgent,
        windowSize: `${window.innerWidth}x${window.innerHeight}`,
        location: window.location.href,
        environment: isDevelopment ? 'development' : 'production',
        siteUrl: siteConfig.url,
        baseUrl: siteConfig.baseUrl,
        timestamp: new Date().toISOString()
      });
    }
  }, [siteConfig, isDevelopment]);

  // Debug info is hidden - return null to not render anything
  return null;
};

export default DebugInfo;