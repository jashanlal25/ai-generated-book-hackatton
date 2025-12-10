import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

export default function Chatbot(): ReactNode {
  return (
    <Layout
      title="AI Chatbot"
      description="Chat with our AI assistant about humanoid robotics">
      <main style={{padding: '2rem', maxWidth: '800px', margin: '0 auto'}}>
        <Heading as="h1">AI Chatbot</Heading>
        <p>
          Ask questions about ROS 2, humanoid robotics, simulation, and
          vision-language-action systems.
        </p>
        <div style={{
          border: '2px dashed #ccc',
          borderRadius: '8px',
          padding: '3rem',
          textAlign: 'center',
          marginTop: '2rem',
          backgroundColor: 'var(--ifm-background-surface-color)'
        }}>
          <p style={{fontSize: '1.2rem', color: 'var(--ifm-color-emphasis-600)'}}>
            Chatbot coming soon...
          </p>
          <p style={{color: 'var(--ifm-color-emphasis-500)'}}>
            This feature will be integrated in a future update.
          </p>
        </div>
      </main>
    </Layout>
  );
}
