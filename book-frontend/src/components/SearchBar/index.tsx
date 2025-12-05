import React, { useState, useEffect, useRef } from 'react';
import { FaSearch, FaTimes } from 'react-icons/fa';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type SearchResult = {
  title: string;
  type: 'module' | 'lecture';
  path: string;
  description: string;
  module?: string;
};

const searchData: SearchResult[] = [
  // Module 1
  {
    title: 'Introduction to Physical AI',
    type: 'module',
    path: '/docs/module-1/intro',
    description: 'Master the robotic nervous system with ROS 2 middleware'
  },
  {
    title: 'ROS 2 Fundamentals',
    type: 'lecture',
    path: '/docs/module-1/lecture-1',
    description: 'Learn ROS 2 basics, nodes, topics, and services',
    module: 'Module 1'
  },
  {
    title: 'Python Robotics Programming',
    type: 'lecture',
    path: '/docs/module-1/lecture-2',
    description: 'Python agents with rospy and robotics libraries',
    module: 'Module 1'
  },
  
  // Module 2
  {
    title: 'ROS 2 - The Robot Operating System',
    type: 'module',
    path: '/docs/module-2/intro',
    description: 'Build physics simulations and high-fidelity environments'
  },
  {
    title: 'Gazebo Physics Simulation',
    type: 'lecture',
    path: '/docs/module-2/lecture-1',
    description: 'Create realistic physics simulations for robots',
    module: 'Module 2'
  },
  {
    title: 'Unity Visualization',
    type: 'lecture',
    path: '/docs/module-2/lecture-2',
    description: 'Build stunning robot visualizations with Unity',
    module: 'Module 2'
  },
  
  // Module 3
  {
    title: 'Robot Simulation and Digital Twins',
    type: 'module',
    path: '/docs/module-3/intro',
    description: 'Advanced perception, navigation, and sim-to-real transfer'
  },
  {
    title: 'Isaac Sim & Synthetic Data',
    type: 'lecture',
    path: '/docs/module-3/lecture-1',
    description: 'Generate synthetic training data with NVIDIA Isaac Sim',
    module: 'Module 3'
  },
  {
    title: 'VSLAM & Navigation',
    type: 'lecture',
    path: '/docs/module-3/lecture-2',
    description: 'Visual SLAM and autonomous navigation systems',
    module: 'Module 3'
  },
  
  // Module 4
  {
    title: 'AI-Powered Robot Perception',
    type: 'module',
    path: '/docs/module-4/intro',
    description: 'Convergence of LLM and Robotics for conversational control'
  },
  {
    title: 'Voice to Action (Whisper)',
    type: 'lecture',
    path: '/docs/module-4/lecture-1',
    description: 'Convert speech to robot actions using Whisper AI',
    module: 'Module 4'
  },
  {
    title: 'LLM Cognitive Planning',
    type: 'lecture',
    path: '/docs/module-4/lecture-2',
    description: 'Large Language Models for robot task planning',
    module: 'Module 4'
  },
  
  // Module 5
  {
    title: 'Human-Robot Interaction',
    type: 'module',
    path: '/docs/module-5/intro',
    description: 'Advanced multimodal AI for natural human-robot communication'
  },
  {
    title: 'Vision-Language Models',
    type: 'lecture',
    path: '/docs/module-5/lecture-1',
    description: 'Multimodal AI combining vision and language understanding',
    module: 'Module 5'
  },
  {
    title: 'Gesture Recognition',
    type: 'lecture',
    path: '/docs/module-5/lecture-2',
    description: 'Recognize and interpret human gestures for robot control',
    module: 'Module 5'
  }
];

export default function SearchBar() {
  const [query, setQuery] = useState('');
  const [results, setResults] = useState<SearchResult[]>([]);
  const [isOpen, setIsOpen] = useState(false);
  const searchRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (query.length > 0) {
      const filtered = searchData.filter(item =>
        item.title.toLowerCase().includes(query.toLowerCase()) ||
        item.description.toLowerCase().includes(query.toLowerCase()) ||
        (item.module && item.module.toLowerCase().includes(query.toLowerCase()))
      );
      setResults(filtered);
      setIsOpen(true);
    } else {
      setResults([]);
      setIsOpen(false);
    }
  }, [query]);

  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (searchRef.current && !searchRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    }

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleClear = () => {
    setQuery('');
    setResults([]);
    setIsOpen(false);
  };

  return (
    <div className={styles.searchContainer} ref={searchRef}>
      <div className={styles.searchInputWrapper}>
        <FaSearch className={styles.searchIcon} />
        <input
          type="text"
          placeholder="Search modules & lectures..."
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          className={styles.searchInput}
          onFocus={() => query.length > 0 && setIsOpen(true)}
        />
        {query && (
          <button onClick={handleClear} className={styles.clearButton}>
            <FaTimes />
          </button>
        )}
      </div>
      
      {isOpen && results.length > 0 && (
        <div className={styles.searchResults}>
          {results.map((result, index) => (
            <Link
              key={index}
              to={result.path}
              className={styles.searchResultItem}
              onClick={() => setIsOpen(false)}
            >
              <div className={styles.resultHeader}>
                <span className={styles.resultTitle}>{result.title}</span>
                <span className={`${styles.resultType} ${styles[result.type]}`}>
                  {result.type === 'module' ? 'Module' : 'Lecture'}
                </span>
              </div>
              <div className={styles.resultDescription}>{result.description}</div>
              {result.module && (
                <div className={styles.resultModule}>{result.module}</div>
              )}
            </Link>
          ))}
        </div>
      )}
      
      {isOpen && query.length > 0 && results.length === 0 && (
        <div className={styles.searchResults}>
          <div className={styles.noResults}>
            No results found for "{query}"
          </div>
        </div>
      )}
    </div>
  );
}