import React from 'react';

interface AvatarProps {
  src?: string;
  alt?: string;
  className?: string;
  fallback?: string;
}

const Avatar: React.FC<AvatarProps> = ({ src, alt, className = '', fallback }) => {
  const [hasError, setHasError] = React.useState(false);

  return (
    <div className={`rounded-full overflow-hidden flex items-center justify-center ${className}`}>
      {src && !hasError ? (
        <img
          src={src}
          alt={alt || ''}
          className="w-full h-full object-cover"
          onError={() => setHasError(true)}
        />
      ) : (
        <div className="w-full h-full bg-gray-200 flex items-center justify-center">
          {fallback || (alt?.charAt(0)?.toUpperCase() || '?')}
        </div>
      )}
    </div>
  );
};

export default Avatar;