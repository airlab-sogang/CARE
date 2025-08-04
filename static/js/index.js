window.HELP_IMPROVE_VIDEOJS = false;

var INTERP_BASE = "./static/interpolation/stacked";
var NUM_INTERP_FRAMES = 240;

var interp_images = [];
function preloadInterpolationImages() {
  for (var i = 0; i < NUM_INTERP_FRAMES; i++) {
    var path = INTERP_BASE + '/' + String(i).padStart(6, '0') + '.jpg';
    interp_images[i] = new Image();
    interp_images[i].src = path;
  }
}

function setInterpolationImage(i) {
  var image = interp_images[i];
  image.ondragstart = function() { return false; };
  image.oncontextmenu = function() { return false; };
  $('#interpolation-image-wrapper').empty().append(image);
}


$(document).ready(function() {
    // Check for click events on the navbar burger icon
    $(".navbar-burger").click(function() {
      // Toggle the "is-active" class on both the "navbar-burger" and the "navbar-menu"
      $(".navbar-burger").toggleClass("is-active");
      $(".navbar-menu").toggleClass("is-active");

    });

    var options = {
			slidesToScroll: 1,
			slidesToShow: 3,
			loop: true,
			infinite: true,
			autoplay: false,
			autoplaySpeed: 3000,
    }

		// Initialize all div with carousel class
    var carousels = bulmaCarousel.attach('.carousel', options);

    // Loop on each carousel initialized
    for(var i = 0; i < carousels.length; i++) {
    	// Add listener to  event
    	carousels[i].on('before:show', state => {
    		console.log(state);
    	});
    }

    // Access to bulmaCarousel instance of an element
    var element = document.querySelector('#my-element');
    if (element && element.bulmaCarousel) {
    	// bulmaCarousel instance is available as element.bulmaCarousel
    	element.bulmaCarousel.on('before-show', function(state) {
    		console.log(state);
    	});
    }

    /*var player = document.getElementById('interpolation-video');
    player.addEventListener('loadedmetadata', function() {
      $('#interpolation-slider').on('input', function(event) {
        console.log(this.value, player.duration);
        player.currentTime = player.duration / 100 * this.value;
      })
    }, false);*/
    preloadInterpolationImages();

    $('#interpolation-slider').on('input', function(event) {
      setInterpolationImage(this.value);
    });
    setInterpolationImage(0);
    $('#interpolation-slider').prop('max', NUM_INTERP_FRAMES - 1);

    bulmaSlider.attach();

    // Robot Carousel Functionality
    function initRobotCarousels() {
        const robots = ['turtlebot4', 'locobot', 'robomaster'];
        
        // Handle Exploration section
        const explorationTabs = document.querySelectorAll('.exploration-tab');
        const explorationSlides = document.querySelectorAll('.exploration-slide');
        const explorationPrev = document.querySelector('.exploration-prev');
        const explorationNext = document.querySelector('.exploration-next');
        
        let currentExplorationIndex = 0;
        
        function showExplorationSlide(index) {
            // Hide all slides
            explorationSlides.forEach(slide => {
                slide.style.display = 'none';
                slide.classList.remove('active');
            });
            
            // Remove active class from all tabs
            explorationTabs.forEach(tab => tab.classList.remove('is-active'));
            
            // Show current slide and activate tab
            if (explorationSlides[index]) {
                explorationSlides[index].style.display = 'block';
                explorationSlides[index].classList.add('active');
                explorationTabs[index].classList.add('is-active');
            }
        }
        
        // Exploration tab clicks
        explorationTabs.forEach((tab, index) => {
            tab.addEventListener('click', (e) => {
                e.preventDefault();
                currentExplorationIndex = index;
                showExplorationSlide(index);
            });
        });
        
        // Exploration navigation arrows
        if (explorationPrev) {
            explorationPrev.addEventListener('click', () => {
                currentExplorationIndex = (currentExplorationIndex - 1 + robots.length) % robots.length;
                showExplorationSlide(currentExplorationIndex);
            });
        }
        
        if (explorationNext) {
            explorationNext.addEventListener('click', () => {
                currentExplorationIndex = (currentExplorationIndex + 1) % robots.length;
                showExplorationSlide(currentExplorationIndex);
            });
        }
        
        // Handle Navigation section
        const navigationTabs = document.querySelectorAll('.navigation-tab');
        const navigationSlides = document.querySelectorAll('.navigation-slide');
        const navigationPrev = document.querySelector('.navigation-prev');
        const navigationNext = document.querySelector('.navigation-next');
        
        let currentNavigationIndex = 0;
        
        function showNavigationSlide(index) {
            // Hide all slides
            navigationSlides.forEach(slide => {
                slide.style.display = 'none';
                slide.classList.remove('active');
            });
            
            // Remove active class from all tabs
            navigationTabs.forEach(tab => tab.classList.remove('is-active'));
            
            // Show current slide and activate tab
            if (navigationSlides[index]) {
                navigationSlides[index].style.display = 'block';
                navigationSlides[index].classList.add('active');
                navigationTabs[index].classList.add('is-active');
            }
        }
        
        // Navigation tab clicks
        navigationTabs.forEach((tab, index) => {
            tab.addEventListener('click', (e) => {
                e.preventDefault();
                currentNavigationIndex = index;
                showNavigationSlide(index);
            });
        });
        
        // Navigation navigation arrows
        if (navigationPrev) {
            navigationPrev.addEventListener('click', () => {
                currentNavigationIndex = (currentNavigationIndex - 1 + robots.length) % robots.length;
                showNavigationSlide(currentNavigationIndex);
            });
        }
        
        if (navigationNext) {
            navigationNext.addEventListener('click', () => {
                currentNavigationIndex = (currentNavigationIndex + 1) % robots.length;
                showNavigationSlide(currentNavigationIndex);
            });
        }
        
        // Initialize with first slide visible
        showExplorationSlide(0);
        showNavigationSlide(0);
    }
    
    // Initialize robot carousels
    initRobotCarousels();

})
