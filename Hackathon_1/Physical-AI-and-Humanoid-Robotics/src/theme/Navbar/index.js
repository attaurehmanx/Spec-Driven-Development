import React from 'react';
import Navbar from '@theme-original/Navbar';
import Authentication from '@site/src/components/Authentication';
import { useLocation } from '@docusaurus/router';

function NavbarWrapper(props) {
  const location = useLocation();

  // Don't show authentication UI on sign-in or sign-up pages
  const shouldShowAuth = !location.pathname.startsWith('/sign-in') && !location.pathname.startsWith('/sign-up');

  return (
    <div style={{ position: 'relative' }}>
      <Navbar {...props} />
      {shouldShowAuth && (
        <div style={{
          position: 'absolute',
          right: '150px', // Positioned to the left of GitHub and dark mode icons
          top: '50%',
          transform: 'translateY(-50%)',
          zIndex: 1000,
          display: 'flex',
          alignItems: 'center',
          height: '100%'
        }}>
          <Authentication />
        </div>
      )}
    </div>
  );
}

export default NavbarWrapper;