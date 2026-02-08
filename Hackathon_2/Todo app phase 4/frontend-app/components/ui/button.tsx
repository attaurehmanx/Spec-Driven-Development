import * as React from "react";
import { Slot } from "@radix-ui/react-slot";
import { cva, type VariantProps } from "class-variance-authority";

import { cn } from "@/lib/utils";

const buttonVariants = cva(
  "inline-flex items-center justify-center gap-2 whitespace-nowrap rounded-xl text-sm font-bold transition-all duration-200 focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2 disabled:pointer-events-none disabled:opacity-50 [&_svg]:pointer-events-none [&_svg]:size-4 [&_svg]:shrink-0 active:scale-[0.95] hover:scale-[1.05] hover:-translate-y-1",
  {
    variants: {
      variant: {
        default:
          "bg-gradient-to-r from-primary-600 to-primary-500 !text-white shadow-xl shadow-primary-500/50 hover:shadow-2xl hover:shadow-primary-500/60 hover:from-primary-500 hover:to-primary-400 dark:from-primary-600 dark:to-primary-500 dark:hover:from-primary-500 dark:hover:to-primary-400",
        destructive:
          "bg-gradient-to-r from-red-600 to-red-500 text-white shadow-xl shadow-red-500/50 hover:shadow-2xl hover:shadow-red-500/60 hover:from-red-500 hover:to-red-400",
        outline:
          "border-2 border-primary-500 bg-transparent text-primary-600 dark:text-primary-400 shadow-lg hover:bg-primary-500 hover:text-white hover:shadow-xl hover:shadow-primary-500/40",
        secondary:
          "bg-gradient-to-r from-secondary-600 to-secondary-500 text-white shadow-xl shadow-secondary-500/50 hover:shadow-2xl hover:shadow-secondary-500/60 hover:from-secondary-500 hover:to-secondary-400",
        success:
          "bg-gradient-to-r from-success-600 to-success-500 text-white shadow-xl shadow-success-500/50 hover:shadow-2xl hover:shadow-success-500/60 hover:from-success-500 hover:to-success-400",
        ghost: "hover:bg-primary-500/10 hover:text-primary-600 dark:hover:text-primary-400",
        link: "text-primary-600 dark:text-primary-400 underline-offset-4 hover:underline hover:scale-100 hover:translate-y-0",
      },
      size: {
        default: "h-11 px-6 py-2.5",
        sm: "h-9 rounded-lg px-4 text-xs",
        lg: "h-14 rounded-2xl px-10 text-base",
        icon: "h-11 w-11",
      },
    },
    defaultVariants: {
      variant: "default",
      size: "default",
    },
  }
);

export interface ButtonProps
  extends React.ButtonHTMLAttributes<HTMLButtonElement>,
    VariantProps<typeof buttonVariants> {
  asChild?: boolean;
  isLoading?: boolean;
}

const Button = React.forwardRef<HTMLButtonElement, ButtonProps>(
  ({ className, variant, size, asChild = false, isLoading, children, ...props }, ref) => {
    const Comp = asChild ? Slot : "button";

    // If loading, disable the button and show spinner
    if (isLoading) {
      return (
        <Comp
          className={cn(buttonVariants({ variant, size, className }))}
          ref={ref}
          disabled
          {...props}
        >
          <svg
            className="animate-spin -ml-1 mr-2 h-4 w-4 text-current"
            xmlns="http://www.w3.org/2000/svg"
            fill="none"
            viewBox="0 0 24 24"
          >
            <circle
              className="opacity-25"
              cx="12"
              cy="12"
              r="10"
              stroke="currentColor"
              strokeWidth="4"
            ></circle>
            <path
              className="opacity-75"
              fill="currentColor"
              d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
            ></path>
          </svg>
          {children}
        </Comp>
      );
    }

    return (
      <Comp
        className={cn(buttonVariants({ variant, size, className }))}
        ref={ref}
        {...props}
      >
        {children}
      </Comp>
    );
  }
);
Button.displayName = "Button";

export { Button, buttonVariants };