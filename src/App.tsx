import { useEffect, lazy, Suspense } from "react";
import { ReactLenis } from "lenis/react";
import Nav from "@components/layout/Navigation/Nav";
import Hero from "@components/layout/Header/Hero";
import Footer from "@components/layout/Footer/Footer";
import AmbientBackground from "@components/layout/AmbientBackground";
import ErrorBoundary from "@components/common/ErrorBoundary";
import ScrollProgress from "@components/ui/ScrollProgress";
import BackToTop from "@components/ui/BackToTop";
import Preloader from "@components/ui/Preloader";
import KeyboardNav from "@components/ui/KeyboardNav";
import SectionLoader from "@components/ui/SectionLoader";
import ScrollDiagnostic, {
   type ScrollDebugMode,
} from "@components/ui/ScrollDiagnostic";
import { BreakpointProvider } from "@hooks/BreakpointProvider";

// Lazy Load "Below the fold" sections for massive performance gains
const About = lazy(() => import("@pages/about/About"));
const Experience = lazy(() => import("@pages/experience/Experience"));
const Skill = lazy(() => import("@pages/skill/Skill"));
const Education = lazy(() => import("@pages/education/Education"));
const Portfolio = lazy(() => import("@pages/portfolio/Portfolio"));
const Contact = lazy(() => import("@pages/contact/Contact"));

const getDebugMode = (): ScrollDebugMode | null => {
   const mode = new URLSearchParams(globalThis.location.search).get("scroll-debug");
   return mode === "bare" || mode === "no-lenis" || mode === "no-overlays" || mode === "full"
      ? mode
      : null;
};

const SiteContent = ({ includeOverlays }: { includeOverlays: boolean }) => (
   <BreakpointProvider>
      <ErrorBoundary>
         {includeOverlays && <Preloader />}
         {includeOverlays && <ScrollProgress />}
         <KeyboardNav />
         {includeOverlays && <AmbientBackground />}
         <div className="relative min-h-screen">
            <a href="#main-content" className="skip-link">
               Skip to content
            </a>
            <Nav />
            <main id="main-content" tabIndex={-1}>
               <Hero />
               <Suspense fallback={<SectionLoader />}>
                  <div className="section-darker">
                     <About />
                  </div>
                  <div className="section-dark">
                     <Experience />
                  </div>
                  <div className="section-darker">
                     <Education />
                  </div>
                  <div className="section-dark">
                     <Skill />
                  </div>
                  <div className="section-darker" id="projects">
                     <Portfolio />
                  </div>
                  <div className="section-darker" id="contact">
                     <Contact />
                  </div>
               </Suspense>
            </main>
            <Footer />
            <BackToTop />
            {/* <SystemStatus /> */}
         </div>
      </ErrorBoundary>
   </BreakpointProvider>
);

const App = () => {
   const debugMode = getDebugMode();

   useEffect(() => {
      globalThis.history.scrollRestoration = "manual";
      globalThis.scrollTo(0, 0);
   }, []);

   if (debugMode === "bare") {
      return (
         <ScrollDiagnostic mode={debugMode}>
            <div style={{ minHeight: "300vh", background: "#0b1012" }} />
         </ScrollDiagnostic>
      );
   }

   const content = <SiteContent includeOverlays={debugMode !== "no-overlays"} />;
   const withDiagnostic = debugMode ? (
      <ScrollDiagnostic mode={debugMode}>{content}</ScrollDiagnostic>
   ) : (
      content
   );

   if (debugMode === "no-lenis" || debugMode === "no-overlays") {
      return withDiagnostic;
   }

   return (
      <ReactLenis
         root
         options={{
            lerp: 0.1,
            wheelMultiplier: 1.1,
            touchMultiplier: 1.5,
            syncTouch: false,
         }}
      >
         {withDiagnostic}
      </ReactLenis>
   );
};

export default App;
