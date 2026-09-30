"use strict";
const play = document.getElementById("flexure-play");
const figure = document.getElementById("flexure-animation");
const gif = document.getElementById("flexure-gif");
const status = document.getElementById("flexure-media-status");
if (play && figure && gif && status) {
  play.hidden = false;
  gif.addEventListener("load", () => { status.textContent = ""; });
  gif.addEventListener("error", () => { status.textContent = "The animation could not load. Try the animation file link."; });
  play.addEventListener("click", () => {
    const opening = figure.hidden;
    figure.hidden = !opening;
    play.setAttribute("aria-expanded", String(opening));
    play.textContent = opening ? "Stop animation" : "Play jaw animation";
    if (opening) {
      status.textContent = "Loading jaw animation…";
      gif.src = "micro-flexure/jaw-preview.gif";
    } else {
      gif.removeAttribute("src");
      status.textContent = "";
    }
  });
}
