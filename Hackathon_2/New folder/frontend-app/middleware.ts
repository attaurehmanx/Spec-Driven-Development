import { NextRequest, NextResponse } from 'next/server';

// Define protected routes that require authentication
const protectedRoutes = ['/dashboard', '/dashboard/tasks', '/dashboard/tasks/create'];

// Define public routes that don't require authentication
const publicRoutes = ['/', '/auth/sign-in', '/auth/sign-up'];

export function middleware(request: NextRequest) {
  // Get the token from cookies or headers
  const token = request.cookies.get('better_auth_token')?.value ||
                request.headers.get('authorization')?.replace('Bearer ', '');

  // Check if the current route is protected
  const isProtectedRoute = protectedRoutes.some(route =>
    request.nextUrl.pathname.startsWith(route)
  );

  // Check if the current route is public
  const isPublicRoute = publicRoutes.includes(request.nextUrl.pathname);

  // If the route is protected and user is not authenticated, redirect to sign-in
  if (isProtectedRoute && !token) {
    return NextResponse.redirect(new URL('/auth/sign-in', request.url));
  }

  // If the route is public and user is already authenticated,
  // redirect to dashboard (prevent accessing sign-in/sign-up when logged in)
  if (isPublicRoute && token) {
    const signInPath = request.nextUrl.pathname === '/auth/sign-in';
    const signUpPath = request.nextUrl.pathname === '/auth/sign-up';

    if (signInPath || signUpPath) {
      return NextResponse.redirect(new URL('/dashboard', request.url));
    }
  }

  // Continue with the request
  return NextResponse.next();
}

// Define which paths the middleware should run on
export const config = {
  matcher: [
    /*
     * Match all request paths except for the ones starting with:
     * - api (API routes)
     * - _next/static (static files)
     * - _next/image (image optimization files)
     * - favicon.ico (favicon file)
     */
    '/((?!api|_next/static|_next/image|favicon.ico).*)',
  ],
};