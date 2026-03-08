/* ==========================================
   main.js — Sidebar navigation & scroll effects
   ========================================== */

(function () {
  'use strict';

  // --- Sidebar active link tracking ---
  const sections = document.querySelectorAll('.section');
  const navLinks = document.querySelectorAll('.sidebar-links a');

  function updateActiveLink() {
    let currentId = 'home';
    const scrollY = window.scrollY + window.innerHeight * 0.35;

    sections.forEach(section => {
      if (section.offsetTop <= scrollY) {
        currentId = section.id;
      }
    });

    navLinks.forEach(link => {
      link.classList.toggle('active', link.getAttribute('href') === '#' + currentId);
    });
  }

  // --- Smooth scroll on nav click ---
  navLinks.forEach(link => {
    link.addEventListener('click', function (e) {
      e.preventDefault();
      const targetId = this.getAttribute('href').substring(1);
      const target = document.getElementById(targetId);
      if (target) {
        target.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }
    });
  });

  // --- Section fade-in on scroll ---
  const observer = new IntersectionObserver(
    (entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          entry.target.classList.add('visible');
        }
      });
    },
    { threshold: 0.1, rootMargin: '0px 0px -50px 0px' }
  );

  document.querySelectorAll('.section-inner').forEach(el => {
    observer.observe(el);
  });

  // --- Scroll listener ---
  let ticking = false;
  window.addEventListener('scroll', () => {
    if (!ticking) {
      requestAnimationFrame(() => {
        updateActiveLink();
        ticking = false;
      });
      ticking = true;
    }
  });

  // Initial state
  updateActiveLink();
})();
