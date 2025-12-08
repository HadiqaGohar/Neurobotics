import React, {type ReactNode} from 'react';
import Navbar from '@theme-original/Navbar';
import type NavbarType from '@theme/Navbar';
import type {WrapperProps} from '@docusaurus/types';
// UserMenu will now be rendered via docusaurus.config.ts as a NavbarItem
// import UserMenu from '../../components/UserMenu'; // Removed

type Props = WrapperProps<typeof NavbarType>;

export default function NavbarWrapper(props: Props): ReactNode {
  return (
    <>
      <Navbar {...props} />
      {/* The UserMenu is now integrated as a NavbarItem in docusaurus.config.ts */}
      {/* <div style={{
        position: 'absolute',
        top: '0.5rem',
        right: '1rem',
        zIndex: 1000
      }}>
        <UserMenu />
      </div> */}
    </>
  );
}