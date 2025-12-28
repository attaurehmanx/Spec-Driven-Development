import React from 'react';
import { useLocation } from '@docusaurus/router';
import Authentication from '@site/src/components/Authentication';

export default function CustomNavbarItem() {
  const location = useLocation();

  // Don't show authentication UI on sign-in or sign-up pages
  const shouldShowAuth = !location.pathname.startsWith('/sign-in') && !location.pathname.startsWith('/sign-up');

  if (!shouldShowAuth) {
    return null;
  }

  return (
    <div className="navbar__item navbar__item--right custom-auth-navbar-item">
      <Authentication />
    </div>
  );
}