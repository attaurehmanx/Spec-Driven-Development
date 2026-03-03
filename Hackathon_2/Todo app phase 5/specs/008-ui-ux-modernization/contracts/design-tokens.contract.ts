/**
 * Design Tokens Contract
 *
 * TypeScript type definitions for design tokens (colors, spacing, typography, etc.)
 * that are defined as CSS custom properties and Tailwind configuration.
 */

/**
 * Color Tokens
 *
 * Semantic color names that map to CSS custom properties.
 * Values are HSL format: "hue saturation lightness"
 */
export type ColorToken =
  | 'background'
  | 'foreground'
  | 'card'
  | 'card-foreground'
  | 'popover'
  | 'popover-foreground'
  | 'primary'
  | 'primary-foreground'
  | 'secondary'
  | 'secondary-foreground'
  | 'muted'
  | 'muted-foreground'
  | 'accent'
  | 'accent-foreground'
  | 'destructive'
  | 'destructive-foreground'
  | 'border'
  | 'input'
  | 'ring'

/**
 * Color values in HSL format
 */
export interface ColorValue {
  hue: number
  saturation: number
  lightness: number
}

/**
 * Theme color palette
 */
export interface ThemeColors {
  light: Record<ColorToken, ColorValue>
  dark: Record<ColorToken, ColorValue>
}

/**
 * Spacing Tokens
 *
 * Consistent spacing scale based on 4px base unit
 */
export type SpacingToken =
  | '0'
  | '1'    // 4px
  | '2'    // 8px
  | '3'    // 12px
  | '4'    // 16px
  | '5'    // 20px
  | '6'    // 24px
  | '8'    // 32px
  | '10'   // 40px
  | '12'   // 48px
  | '16'   // 64px
  | '20'   // 80px
  | '24'   // 96px

/**
 * Border Radius Tokens
 */
export type RadiusToken =
  | 'none'   // 0
  | 'sm'     // 0.125rem (2px)
  | 'md'     // 0.375rem (6px)
  | 'lg'     // 0.5rem (8px)
  | 'xl'     // 0.75rem (12px)
  | '2xl'    // 1rem (16px)
  | '3xl'    // 1.5rem (24px)
  | 'full'   // 9999px

/**
 * Shadow Tokens
 */
export type ShadowToken =
  | 'none'
  | 'sm'
  | 'md'
  | 'lg'
  | 'xl'
  | '2xl'
  | 'inner'

/**
 * Shadow values
 */
export interface ShadowValue {
  offsetX: string
  offsetY: string
  blur: string
  spread: string
  color: string
}

/**
 * Typography Tokens
 */
export type FontFamilyToken = 'sans' | 'serif' | 'mono'

export type FontSizeToken =
  | 'xs'    // 0.75rem (12px)
  | 'sm'    // 0.875rem (14px)
  | 'base'  // 1rem (16px)
  | 'lg'    // 1.125rem (18px)
  | 'xl'    // 1.25rem (20px)
  | '2xl'   // 1.5rem (24px)
  | '3xl'   // 1.875rem (30px)
  | '4xl'   // 2.25rem (36px)
  | '5xl'   // 3rem (48px)

export type FontWeightToken =
  | 'thin'       // 100
  | 'extralight' // 200
  | 'light'      // 300
  | 'normal'     // 400
  | 'medium'     // 500
  | 'semibold'   // 600
  | 'bold'       // 700
  | 'extrabold'  // 800
  | 'black'      // 900

export type LineHeightToken =
  | 'none'     // 1
  | 'tight'    // 1.25
  | 'snug'     // 1.375
  | 'normal'   // 1.5
  | 'relaxed'  // 1.625
  | 'loose'    // 2

/**
 * Animation Duration Tokens
 */
export type DurationToken =
  | 'fast'    // 150ms
  | 'normal'  // 300ms
  | 'slow'    // 400ms

/**
 * Animation Easing Tokens
 */
export type EasingToken =
  | 'linear'
  | 'ease-in'
  | 'ease-out'
  | 'ease-in-out'

/**
 * Z-Index Tokens
 *
 * Layering system for stacking contexts
 */
export type ZIndexToken =
  | 'base'      // 0
  | 'dropdown'  // 1000
  | 'sticky'    // 1100
  | 'fixed'     // 1200
  | 'modal'     // 1300
  | 'popover'   // 1400
  | 'tooltip'   // 1500

/**
 * Breakpoint Tokens
 *
 * Responsive design breakpoints
 */
export type BreakpointToken =
  | 'sm'   // 640px
  | 'md'   // 768px
  | 'lg'   // 1024px
  | 'xl'   // 1280px
  | '2xl'  // 1536px

/**
 * Complete design token system
 */
export interface DesignTokens {
  colors: ThemeColors
  spacing: Record<SpacingToken, string>
  radius: Record<RadiusToken, string>
  shadows: Record<ShadowToken, ShadowValue | string>
  typography: {
    fontFamily: Record<FontFamilyToken, string[]>
    fontSize: Record<FontSizeToken, string>
    fontWeight: Record<FontWeightToken, number>
    lineHeight: Record<LineHeightToken, number>
  }
  animation: {
    duration: Record<DurationToken, string>
    easing: Record<EasingToken, string>
  }
  zIndex: Record<ZIndexToken, number>
  breakpoints: Record<BreakpointToken, string>
}

/**
 * CSS Custom Property Names
 *
 * Maps token names to CSS variable names
 */
export const CSSVariables = {
  // Colors
  background: '--background',
  foreground: '--foreground',
  card: '--card',
  cardForeground: '--card-foreground',
  primary: '--primary',
  primaryForeground: '--primary-foreground',
  muted: '--muted',
  mutedForeground: '--muted-foreground',
  accent: '--accent',
  accentForeground: '--accent-foreground',
  border: '--border',
  input: '--input',
  ring: '--ring',

  // Spacing
  radius: '--radius',

  // Shadows
  shadowSm: '--shadow-sm',
  shadowMd: '--shadow-md',
  shadowLg: '--shadow-lg',

  // Animation
  durationFast: '--duration-fast',
  durationNormal: '--duration-normal',
  durationSlow: '--duration-slow',
} as const

/**
 * Tailwind Class Utilities
 *
 * Type-safe Tailwind class name builders
 */
export type TailwindColorClass = `bg-${ColorToken}` | `text-${ColorToken}` | `border-${ColorToken}`
export type TailwindSpacingClass = `p-${SpacingToken}` | `m-${SpacingToken}` | `gap-${SpacingToken}`
export type TailwindRadiusClass = `rounded-${RadiusToken}`
export type TailwindShadowClass = `shadow-${ShadowToken}`

/**
 * Example usage:
 *
 * ```tsx
 * // Using CSS custom properties
 * const styles = {
 *   backgroundColor: 'hsl(var(--background))',
 *   color: 'hsl(var(--foreground))',
 *   borderRadius: 'var(--radius)',
 * }
 *
 * // Using Tailwind classes
 * <div className="bg-background text-foreground rounded-xl shadow-lg p-4">
 *   Content
 * </div>
 *
 * // Using design tokens in TypeScript
 * const buttonPadding: SpacingToken = '4'
 * const buttonRadius: RadiusToken = 'lg'
 * const animationDuration: DurationToken = 'normal'
 *
 * // Type-safe class composition
 * import { cn } from '@/lib/utils'
 *
 * const className = cn(
 *   'bg-primary text-primary-foreground',
 *   'rounded-lg shadow-md',
 *   'p-4 m-2',
 *   'transition-all duration-normal'
 * )
 * ```
 */

/**
 * Contrast Ratio Requirements (WCAG AA)
 */
export interface ContrastRequirements {
  normalText: 4.5  // Minimum contrast ratio for normal text
  largeText: 3.0   // Minimum contrast ratio for large text (18pt+ or 14pt+ bold)
  uiComponents: 3.0 // Minimum contrast ratio for UI components
}

/**
 * Accessibility Utilities
 */
export interface AccessibilityTokens {
  /**
   * Focus ring configuration
   */
  focusRing: {
    width: string
    color: ColorToken
    offset: string
  }

  /**
   * Reduced motion preferences
   */
  reducedMotion: {
    enabled: boolean
    fallbackDuration: DurationToken
  }

  /**
   * Minimum touch target size (44x44px per WCAG)
   */
  minTouchTarget: {
    width: string
    height: string
  }
}

/**
 * Export all token types
 */
export type DesignToken =
  | ColorToken
  | SpacingToken
  | RadiusToken
  | ShadowToken
  | FontSizeToken
  | FontWeightToken
  | DurationToken
  | EasingToken
  | ZIndexToken
  | BreakpointToken
