import { useEffect, useState, type ReactNode } from "react";

export type ScrollDebugMode = "bare" | "no-lenis" | "no-overlays" | "full";

interface ScrollDiagnosticProps {
   mode: ScrollDebugMode;
   children: ReactNode;
}

interface ScrollState {
   event: string;
   scrollTop: number;
   scrollHeight: number;
   viewportHeight: number;
   htmlOverflow: string;
   bodyOverflow: string;
}

const getScrollState = (event: string): ScrollState => {
   const scrollingElement = document.scrollingElement;
   return {
      event,
      scrollTop: Math.round(scrollingElement?.scrollTop ?? 0),
      scrollHeight: Math.round(scrollingElement?.scrollHeight ?? 0),
      viewportHeight: Math.round(globalThis.innerHeight),
      htmlOverflow: globalThis.getComputedStyle(document.documentElement).overflow,
      bodyOverflow: globalThis.getComputedStyle(document.body).overflow,
   };
};

const ScrollDiagnostic = ({ mode, children }: ScrollDiagnosticProps) => {
   const [state, setState] = useState(() => getScrollState("waiting"));

   useEffect(() => {
      let frame = 0;
      const update = (event: string) => {
         cancelAnimationFrame(frame);
         frame = requestAnimationFrame(() => setState(getScrollState(event)));
      };

      const onTouch = (event: TouchEvent) => {
         const target = event.target instanceof HTMLElement ? event.target.tagName : "node";
         update(`${event.type}:${event.touches.length}:${target}`);
      };
      const onScroll = () => update("scroll");

      document.addEventListener("touchstart", onTouch, { capture: true, passive: true });
      document.addEventListener("touchmove", onTouch, { capture: true, passive: true });
      window.addEventListener("scroll", onScroll, { passive: true });
      update("ready");

      return () => {
         cancelAnimationFrame(frame);
         document.removeEventListener("touchstart", onTouch, true);
         document.removeEventListener("touchmove", onTouch, true);
         window.removeEventListener("scroll", onScroll);
      };
   }, []);

   return (
      <>
         {children}
         <output
            aria-live="polite"
            style={{
               position: "fixed",
               right: 8,
               bottom: 8,
               zIndex: 9999,
               maxWidth: "calc(100vw - 16px)",
               padding: 8,
               border: "1px solid #475569",
               borderRadius: 4,
               background: "#020617e6",
               color: "#e2e8f0",
               fontFamily: "ui-monospace, monospace",
               fontSize: 10,
               lineHeight: 1.4,
               pointerEvents: "none",
            }}
         >
            <div>mode={mode}</div>
            <div>event={state.event}</div>
            <div>
               top={state.scrollTop} height={state.scrollHeight} view={state.viewportHeight}
            </div>
            <div>html={state.htmlOverflow} body={state.bodyOverflow}</div>
         </output>
      </>
   );
};

export default ScrollDiagnostic;
