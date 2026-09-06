import { motion } from "motion/react";
const avatarImage = `${import.meta.env.BASE_URL}IMG_0938.png`;

const DevAvatar = () => {
   return (
      <motion.div
         style={{
            width: 280,
            height: 360,
            position: "relative",
            margin: "0 auto",
            borderRadius: 24,
            overflow: "hidden",
            background: "rgba(10, 10, 20, 0.85)",
            border: "1px solid rgba(255,255,255,0.08)",
            boxShadow: "0 18px 50px rgba(0, 0, 0, 0.28)",
         }}
         initial={{ opacity: 0, y: 12, scale: 0.96 }}
         animate={{ opacity: 1, y: 0, scale: 1 }}
         transition={{ duration: 0.7, delay: 0.2 }}
      >
         <img
            src={avatarImage}
            alt="Alessandro Gorla"
            style={{
               width: "100%",
               height: "100%",
               objectFit: "cover",
               objectPosition: "center center",
               display: "block",
            }}
         />

         <div
            aria-hidden="true"
            style={{
               position: "absolute",
               inset: 0,
               background:
                  "linear-gradient(180deg, rgba(255,255,255,0.02), rgba(0,0,0,0.16))",
               pointerEvents: "none",
            }}
         >
         </div>
      </motion.div>
   );
};

export default DevAvatar;
