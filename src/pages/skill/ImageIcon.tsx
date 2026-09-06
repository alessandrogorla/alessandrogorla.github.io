import type { CSSProperties, ComponentType } from "react";

interface ImageIconProps {
   size?: number;
   style?: CSSProperties;
}

const createImageIcon = (
   src: string,
   displayName: string,
): ComponentType<ImageIconProps> => {
   const ImageIcon = ({ size = 20, style }: ImageIconProps) => (
      <img
         src={src}
         alt=""
         width={size}
         height={size}
         style={{ width: size, height: size, objectFit: "contain", ...style }}
      />
   );
   ImageIcon.displayName = displayName;
   return ImageIcon;
};

export default createImageIcon;
