import React from 'react';
import Navbar from '@theme-original/Navbar';
import Authentication from '@site/src/components/Authentication';
import { useLocation } from '@docusaurus/router';

function NavbarWrapper(props) {
  const location = useLocation();

  // Don't show authentication UI on sign-in or sign-up pages
  const shouldShowAuth = !location.pathname.startsWith('/sign-in') && !location.pathname.startsWith('/sign-up');

  return (
    <div className="navbar-wrapper">
      <Navbar {...props} />
      {shouldShowAuth && (
        <div className="auth-overlay">
          <Authentication />
        </div>
      )}
    </div>
  );
}

export default NavbarWrapper;