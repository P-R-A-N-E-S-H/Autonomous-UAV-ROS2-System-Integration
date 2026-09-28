/**
 * review_slides.js
 * Interactive Presentation Slide Controller for Review Defense.
 * Role: Member 3 - ROS2 & System Integration Engineer.
 */

let currentSlide = 1;
const totalSlides = 6;

function updateSlideUI() {
  document.querySelectorAll('.slide').forEach((slide, idx) => {
    slide.classList.toggle('active-slide', idx + 1 === currentSlide);
  });
  document.getElementById('current-slide-num').textContent = currentSlide;
  document.getElementById('total-slides-num').textContent = totalSlides;
}

function nextSlide() {
  if (currentSlide < totalSlides) {
    currentSlide++;
    updateSlideUI();
  }
}

function prevSlide() {
  if (currentSlide > 1) {
    currentSlide--;
    updateSlideUI();
  }
}

function toggleFullScreenDeck() {
  const container = document.querySelector('.slides-container');
  if (!document.fullscreenElement) {
    container.requestFullscreen().catch(err => {
      console.warn(`Error attempting to enable fullscreen: ${err.message}`);
    });
  } else {
    document.exitFullscreen();
  }
}

// Keyboard shortcuts for presentation: Left/Right Arrow
document.addEventListener('keydown', (e) => {
  if (e.key === 'ArrowRight' || e.key === 'Space') {
    nextSlide();
  } else if (e.key === 'ArrowLeft') {
    prevSlide();
  }
});
