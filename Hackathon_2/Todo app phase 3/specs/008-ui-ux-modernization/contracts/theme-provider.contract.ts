/**
 * Theme Provider Contract
 *
 * Defines the interface for the ThemeProvider component that wraps the application
 * and provides theme management functionality via next-themes library.
 */

import { ReactNode } from 'react'

/**
 * Valid theme modes
 */
export type ThemeMode = 'light' | 'dark' | 'system'

/**
 * Props for the ThemeProvider component
 */
export interface ThemeProviderProps {
  /**
   * Child components to be wrapped by the theme provider
   */
  children: ReactNode

  /**
   * HTML attribute to use for theme switching
   * @default 'class'
   */
  attribute?: 'class' | 'data-theme' | 'data-mode'

  /**
   * Default theme to use when no preference is saved
   * @default 'system'
   */
  defaultTheme?: ThemeMode

  /**
   * Enable system theme detection via prefers-color-scheme
   * @default true
   */
  enableSystem?: boolean

  /**
   * Disable automatic theme transitions
   * @default false
   */
  disableTransitionOnChange?: boolean

  /**
   * localStorage key for theme persistence
   * @default 'todo-theme'
   */
  storageKey?: string

  /**
   * Force a specific theme (useful for testing)
   */
  forcedTheme?: ThemeMode

  /**
   * Enable color scheme meta tag
   * @default true
   */
  enableColorScheme?: boolean
}

/**
 * Return type of useTheme hook
 */
export interface UseThemeReturn {
  /**
   * Current theme mode ('light' | 'dark' | 'system')
   */
  theme: ThemeMode | undefined

  /**
   * Set the theme mode
   */
  setTheme: (theme: ThemeMode) => void

  /**
   * Forced theme (if set via forcedTheme prop)
   */
  forcedTheme?: ThemeMode

  /**
   * Resolved theme ('light' | 'dark') - what's actually applied
   */
  resolvedTheme?: 'light' | 'dark'

  /**
   * System theme detected from OS
   */
  systemTheme?: 'light' | 'dark'

  /**
   * Available theme options
   */
  themes: ThemeMode[]
}

/**
 * Example usage:
 *
 * ```tsx
 * // app/layout.tsx
 * import { ThemeProvider } from '@/components/providers/theme-provider'
 *
 * export default function RootLayout({ children }) {
 *   return (
 *     <html lang="en" suppressHydrationWarning>
 *       <body>
 *         <ThemeProvider
 *           attribute="class"
 *           defaultTheme="system"
 *           enableSystem
 *           storageKey="todo-theme"
 *         >
 *           {children}
 *         </ThemeProvider>
 *       </body>
 *     </html>
 *   )
 * }
 *
 * // components/theme-aware-component.tsx
 * 'use client'
 *
 * import { useTheme } from 'next-themes'
 *
 * export function ThemeAwareComponent() {
 *   const { theme, setTheme, resolvedTheme } = useTheme()
 *
 *   return (
 *     <div>
 *       <p>Current theme: {theme}</p>
 *       <p>Resolved theme: {resolvedTheme}</p>
 *       <button onClick={() => setTheme('dark')}>Dark</button>
 *       <button onClick={() => setTheme('light')}>Light</button>
 *     </div>
 *   )
 * }
 * ```
 */
