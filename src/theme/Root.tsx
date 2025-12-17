/**
 * Root component wrapper for Docusaurus
 * Adds the floating chat widget to all pages
 */

import React from 'react';
import FloatingChatWidget from '../components/FloatingChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <FloatingChatWidget />
    </>
  );
}
