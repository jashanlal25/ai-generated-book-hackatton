import type {ReactNode} from 'react';
import Layout from '@theme/Layout';
import { useState, useRef, useEffect } from 'react';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

export default function Chatbot(): ReactNode {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      role: 'assistant',
      content: 'Hello! I\'m your AI assistant. How can I help you today?',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the real backend API
      const API_BASE_URL = 'http://localhost:8000';

      const response = await fetch(`${API_BASE_URL}/api/v1/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: userMessage.content,
          conversation_id: null,
          selected_text: null
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.response,
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: 'Sorry, there was an error processing your message. Please try again. Make sure the backend server is running at http://localhost:8000',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <Layout
      title="AI Chatbot"
      description="Chat with our AI assistant about humanoid robotics">
      <main style={{
        padding: '2rem 2rem 0 2rem',
        maxWidth: '100%',
        margin: '0 auto',
        height: '100vh',
        minHeight: '100vh',
        display: 'flex',
        flexDirection: 'column'
      }}>
        <div style={{
          textAlign: 'center',
          marginBottom: '2rem',
          position: 'sticky',
          top: '0',
          backgroundColor: 'var(--ifm-background-color)',
          padding: '1rem 0',
          zIndex: '100',
          boxShadow: '0 2px 4px rgba(0,0,0,0.1)'
        }}>
          <h1 style={{
            fontSize: '2.5rem',
            fontWeight: '700',
            color: '#1a1a1a',
            margin: '0 0 0.5rem 0',
            fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
          }}>
            AI Assistant
          </h1>
          <p style={{
            fontSize: '1.1rem',
            color: '#666',
            margin: 0
          }}>
            Ask questions about ROS 2, humanoid robotics, simulation, and vision-language-action systems
          </p>
        </div>

        {/* Chat Container */}
        <div style={{
          flex: 1,
          display: 'flex',
          flexDirection: 'column',
          maxWidth: '800px',
          margin: '0 auto',
          width: '100%',
          backgroundColor: 'white',
          borderRadius: '16px',
          boxShadow: '0 10px 50px rgba(0, 0, 0, 0.1)',
          overflow: 'hidden',
          border: '1px solid #e2e8f0',
          height: '70vh',
          maxHeight: '70vh'
        }}>
          {/* Messages Area */}
          <div style={{
            flex: 1,
            padding: '1.5rem',
            overflowY: 'auto',
            maxHeight: '50vh',
            minHeight: '40vh',
            display: 'flex',
            flexDirection: 'column',
            gap: '1rem',
            backgroundColor: '#f8fafc'
          }}>
            {messages.map((message) => (
              <div
                key={message.id}
                style={{
                  display: 'flex',
                  justifyContent: message.role === 'user' ? 'flex-end' : 'flex-start',
                  animation: 'fadeIn 0.3s ease-out'
                }}
              >
                <div style={{
                  maxWidth: '80%',
                  padding: '0.8rem 1.2rem',
                  borderRadius: message.role === 'user'
                    ? '18px 6px 18px 18px'
                    : '6px 18px 18px 18px',
                  backgroundColor: message.role === 'user'
                    ? '#667eea'
                    : 'white',
                  color: message.role === 'user' ? 'white' : '#1f2937',
                  boxShadow: '0 2px 8px rgba(0, 0, 0, 0.08)',
                  border: message.role === 'user' ? 'none' : '1px solid #e2e8f0',
                  position: 'relative',
                  wordWrap: 'break-word',
                  fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif'
                }}>
                  <div style={{fontSize: '1rem', lineHeight: '1.5'}}>
                    {message.content}
                  </div>
                  <div style={{
                    fontSize: '0.7rem',
                    color: message.role === 'user' ? 'rgba(255, 255, 255, 0.8)' : '#9ca3af',
                    marginTop: '0.3rem',
                    fontWeight: '500'
                  }}>
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              </div>
            ))}

            {isLoading && (
              <div style={{
                display: 'flex',
                justifyContent: 'flex-start',
                animation: 'fadeIn 0.3s ease-out'
              }}>
                <div style={{
                  maxWidth: '80%',
                  padding: '0.8rem 1.2rem',
                  borderRadius: '6px 18px 18px 18px',
                  backgroundColor: 'white',
                  color: '#1f2937',
                  boxShadow: '0 2px 8px rgba(0, 0, 0, 0.08)',
                  border: '1px solid #e2e8f0',
                  position: 'relative'
                }}>
                  <div style={{
                    display: 'flex',
                    gap: '0.3rem',
                    alignItems: 'center'
                  }}>
                    <div style={{
                      width: '8px',
                      height: '8px',
                      borderRadius: '50%',
                      backgroundColor: '#9ca3af',
                      animation: 'bounce 1.4s infinite ease-in-out both'
                    }}></div>
                    <div style={{
                      width: '8px',
                      height: '8px',
                      borderRadius: '50%',
                      backgroundColor: '#9ca3af',
                      animation: 'bounce 1.4s infinite ease-in-out both',
                      animationDelay: '-0.32s'
                    }}></div>
                    <div style={{
                      width: '8px',
                      height: '8px',
                      borderRadius: '50%',
                      backgroundColor: '#9ca3af',
                      animation: 'bounce 1.4s infinite ease-in-out both',
                      animationDelay: '-0.16s'
                    }}></div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Area */}
          <div style={{
            padding: '1rem',
            backgroundColor: 'white',
            borderTop: '1px solid #e2e8f0',
            display: 'flex',
            gap: '0.75rem'
          }}>
            <form onSubmit={(e) => { e.preventDefault(); handleSendMessage(); }} style={{ flex: 1, display: 'flex' }}>
              <textarea
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyDown={handleKeyDown}
                placeholder="Type your message..."
                rows={1}
                style={{
                  flex: 1,
                  padding: '0.75rem 1rem',
                  border: '1.5px solid #e2e8f0',
                  borderRadius: '20px',
                  fontSize: '1rem',
                  fontFamily: '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
                  resize: 'none',
                  outline: 'none',
                  transition: 'all 0.2s ease',
                  backgroundColor: 'white',
                  boxShadow: 'inset 0 1px 2px rgba(0, 0, 0, 0.05)'
                }}
                disabled={isLoading}
              />
            </form>
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isLoading}
              style={{
                width: '42px',
                height: '42px',
                borderRadius: '50%',
                background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                color: 'white',
                border: 'none',
                fontSize: '16px',
                cursor: 'pointer',
                transition: 'all 0.2s ease',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                boxShadow: '0 2px 10px rgba(102, 126, 234, 0.3)',
                flexShrink: 0
              }}
              onMouseOver={(e) => {
                if (!(!inputValue.trim() || isLoading)) {
                  e.currentTarget.style.transform = 'scale(1.08) rotate(5deg)';
                  e.currentTarget.style.boxShadow = '0 4px 16px rgba(102, 126, 234, 0.4)';
                }
              }}
              onMouseOut={(e) => {
                e.currentTarget.style.transform = 'scale(1)';
                e.currentTarget.style.boxShadow = '0 2px 10px rgba(102, 126, 234, 0.3)';
              }}
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </div>
        </div>

        <style jsx>{`
          @keyframes fadeIn {
            from {
              opacity: 0;
              transform: translateY(8px);
            }
            to {
              opacity: 1;
              transform: translateY(0);
            }
          }

          @keyframes bounce {
            0%, 80%, 100% {
              transform: scale(0);
              opacity: 0.5;
            }
            40% {
              transform: scale(1);
              opacity: 1;
            }
          }
        `}</style>
      </main>
    </Layout>
  );
}
