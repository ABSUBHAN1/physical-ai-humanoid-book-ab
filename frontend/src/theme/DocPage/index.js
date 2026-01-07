import React from 'react';
import clsx from 'clsx';
import { useLocation } from '@docusaurus/router';
import { useDoc } from '@docusaurus/theme-common/internal';
import DocPageLayout from '@theme/DocPage/Layout';
import Translate from '@docusaurus/Translate';

import ChatbotWidget from '@site/src/components/ChatbotWidget/ChatbotWidget';

import styles from './DocPage.module.css';

// This is a custom DocPage component that adds the chatbot widget to chapter pages
export default function DocPage(props) {
  const { content: DocContent } = props;
  const { metadata } = DocContent;
  const { title, description } = metadata;
  const location = useLocation();
  
  // Determine if this is a chapter page based on URL or metadata
  const isChapterPage = location.pathname.includes('/docs/') || 
                       location.pathname.includes('/modules/');

  return (
    <div className={styles.docPage}>
      <DocPageLayout>
        <div className={styles.docContent}>
          <DocContent />
        </div>
      </DocPageLayout>
      
      {/* Show the chatbot widget on chapter pages */}
      {isChapterPage && (
        <div className={styles.chatbotContainer}>
          <ChatbotWidget 
            context={{
              currentPage: location.pathname,
              pageTitle: title,
              pageDescription: description
            }}
          />
        </div>
      )}
    </div>
  );
}