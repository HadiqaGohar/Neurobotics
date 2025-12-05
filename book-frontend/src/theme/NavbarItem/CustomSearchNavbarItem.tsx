import React, {type ReactNode} from 'react';
import SearchBar from '@site/src/components/SearchBar';

export interface Props {
  mobile?: boolean;
  className?: string;
}

export default function CustomSearchNavbarItem({
  mobile,
  className,
}: Props): ReactNode {
  if (mobile) {
    return null;
  }

  return (
    <div className={className} style={{ display: 'flex', alignItems: 'center' }}>
      <SearchBar />
    </div>
  );
}