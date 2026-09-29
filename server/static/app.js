document.addEventListener('DOMContentLoaded', () => {
  const menuToggle = document.querySelector('.menu-toggle');
  const nav = document.querySelector('.main-nav');
  if (menuToggle && nav) {
    menuToggle.addEventListener('click', () => {
      const open = nav.classList.toggle('open');
      menuToggle.setAttribute('aria-expanded', String(open));
    });
  }

  document.querySelectorAll('.alert-close').forEach((button) => {
    button.addEventListener('click', () => button.closest('.alert-banner')?.remove());
  });

  const contactsForm = document.querySelector('#contacts-form');
  const contactsList = document.querySelector('#contact-list');
  const contactTemplate = document.querySelector('#contact-template');
  const contactCount = document.querySelector('#contact-count');
  const addContact = document.querySelector('#add-contact');

  if (contactsForm && contactsList && contactTemplate && contactCount && addContact) {
    const refreshContactFields = () => {
      const cards = [...contactsList.querySelectorAll('[data-contact-card]')];
      contactCount.value = String(cards.length);
      cards.forEach((card, index) => {
        card.querySelector('[data-contact-number]')?.replaceChildren(document.createTextNode(String(index + 1).padStart(2, '0')));
        const title = card.querySelector('.contact-card-title');
        const avatar = card.querySelector('.contact-avatar');
        const name = card.querySelector('[data-field="name"]');
        if (name && title && name.value.trim()) title.textContent = name.value.trim();
        if (name && avatar) avatar.textContent = (name.value.trim()[0] || 'C').toUpperCase();
        card.querySelectorAll('[data-field]').forEach((field) => {
          const key = field.dataset.field === 'phone' ? 'phone' : field.dataset.field;
          field.name = `contact_${key}_${index}`;
        });
      });
    };

    const bindCard = (card) => {
      card.querySelector('.remove-contact')?.addEventListener('click', () => {
        const cards = contactsList.querySelectorAll('[data-contact-card]');
        if (cards.length === 1) {
          card.querySelectorAll('input').forEach((input) => { input.value = ''; });
        } else {
          card.remove();
        }
        refreshContactFields();
      });
      card.querySelector('[data-field="name"]')?.addEventListener('input', refreshContactFields);
    };

    contactsList.querySelectorAll('[data-contact-card]').forEach(bindCard);
    addContact.addEventListener('click', () => {
      const card = contactTemplate.content.cloneNode(true).firstElementChild;
      contactsList.appendChild(card);
      bindCard(card);
      refreshContactFields();
      card.querySelector('input')?.focus();
    });
    contactsForm.addEventListener('submit', refreshContactFields);
    refreshContactFields();
  }

  if (typeof io === 'function') {
    const socket = io();
    socket.on('trigger_reload', (payload) => {
      const destination = new URL(window.location.href);
      destination.pathname = '/main';
      destination.searchParams.set('alert', '1');
      destination.searchParams.set('event', payload?.time || Date.now());
      window.location.assign(destination.toString());
    });
  }
});
