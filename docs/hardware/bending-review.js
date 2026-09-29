"use strict";
const bendingCase = document.getElementById("bending-case");
if (bendingCase) {
  bendingCase.addEventListener("change", () => {
    const image = document.getElementById("bending-case-image");
    image.src = `bending-fit/cases/case-${bendingCase.value}.jpg`;
    image.alt = `Settled validation cable: ${bendingCase.selectedOptions[0].textContent}`;
  });
}
