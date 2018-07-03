/**
 * @author David Gossow - dgossow@willowgarage.com
 */

/**
 * A menu for an interactive marker. This will be overlayed on the canvas.
 *
 * @constructor
 * @param options - object with following keys:
 *
 *  * menuEntries - the menu entries to add
 *  * className (optional) - a custom CSS class for the menu div
 *  * entryClassName (optional) - a custom CSS class for the menu entry
 *  * overlayClassName (optional) - a custom CSS class for the menu overlay
 *  * menuFontSize (optional) - the menu font size
 */
ROS3D.InteractiveMarkerMenu = function(options) {
  THREE.EventDispatcher.call(this);
  var that = this;
  options = options || {};
  var menuEntries = options.menuEntries;
  var className = options.className || 'default-interactive-marker-menu';
  var entryClassName = options.entryClassName || 'default-interactive-marker-menu-entry';
  var overlayClassName = options.overlayClassName || 'default-interactive-marker-overlay';
  var menuFontSize = options.menuFontSize || '0.8em';

  // holds the menu tree
  var allMenus = [];
  allMenus[0] = {
    children : []
  };


  // create the CSS for this marker if it has not been created
  if (document.getElementById('default-interactive-marker-menu-css') === null) {
    var style = document.createElement('style');
    style.id = 'default-interactive-marker-menu-css';
    style.type = 'text/css';
    style.innerHTML = '.default-interactive-marker-menu {' + 'background-color: #444444;'
        + 'border: 1px solid #888888;' + 'border: 1px solid #888888;' + 'padding: 0px 0px 0px 0px;'
        + 'color: #FFFFFF;' + 'font-family: sans-serif;' + 'font-size: ' + menuFontSize +';' + 'z-index: 1002;'
        + '}' + '.default-interactive-marker-menu ul {' + 'padding: 0px 0px 5px 0px;'
        + 'margin: 0px;' + 'list-style-type: none;' + '}'
        + '.default-interactive-marker-menu ul li div {' + '-webkit-touch-callout: none;'
        + '-webkit-user-select: none;' + '-khtml-user-select: none;' + '-moz-user-select: none;'
        + '-ms-user-select: none;' + 'user-select: none;' + 'cursor: default;'
        + 'padding: 3px 10px 3px 10px;' + '}' + '.default-interactive-marker-menu-entry:hover {'
        + '  background-color: #666666;' + '  cursor: pointer;' + '}'
        + '.default-interactive-marker-menu ul ul {' + '  font-style: italic;'
        + '  padding-left: 10px;' + '}' + '.default-interactive-marker-overlay {'
        + '  position: absolute;' + '  top: 0%;' + '  left: 0%;' + '  width: 100%;'
        + '  height: 100%;' + '  background-color: black;' + '  z-index: 1001;'
        + '  -moz-opacity: 0.0;' + '  opacity: .0;' + '  filter: alpha(opacity = 0);' + '}';
    document.getElementsByTagName('head')[0].appendChild(style);
  }

  // place the menu in a div
  this.menuDomElem = document.createElement('div');
  this.menuDomElem.style.position = 'absolute';
  this.menuDomElem.className = className;
  this.menuDomElem.addEventListener('contextmenu', function(event) {
    event.preventDefault();
  });

  // create the overlay DOM
  this.overlayDomElem = document.createElement('div');
  this.overlayDomElem.className = overlayClassName;

  this.hideListener = this.hide.bind(this);
  this.overlayDomElem.addEventListener('contextmenu', this.hideListener);
  this.overlayDomElem.addEventListener('click', this.hideListener);
  this.overlayDomElem.addEventListener('touchstart', this.hideListener);

  // parse all entries and link children to parents
  var i, entry, id;
  for ( i = 0; i < menuEntries.length; i++) {
    entry = menuEntries[i];
    id = entry.id;
    allMenus[id] = {
      title : entry.title,
      id : id,
      children : []
    };
  }
  for ( i = 0; i < menuEntries.length; i++) {
    entry = menuEntries[i];
    id = entry.id;
    var menu = allMenus[id];
    var parent = allMenus[entry.parent_id];
    parent.children.push(menu);
  }

  function emitMenuSelect(menuEntry, domEvent) {
    this.dispatchEvent({
      type : 'menu-select',
      domEvent : domEvent,
      id : menuEntry.id,
      controlName : this.controlName
    });
    this.hide(domEvent);
  }

  /**
   * Create the HTML UL element for the menu and link it to the parent.
   *
   * @param parentDomElem - the parent DOM element
   * @param parentMenu - the parent menu
   */
  function makeUl(parentDomElem, parentMenu) {

    var ulElem = document.createElement('ul');
    parentDomElem.appendChild(ulElem);

    var children = parentMenu.children;

    for ( var i = 0; i < children.length; i++) {
      var liElem = document.createElement('li');
      var divElem = document.createElement('div');
      divElem.appendChild(document.createTextNode(children[i].title));
      ulElem.appendChild(liElem);
      liElem.appendChild(divElem);

      if (children[i].children.length > 0) {
        makeUl(liElem, children[i]);
        divElem.addEventListener('click', that.hide.bind(that));
        divElem.addEventListener('touchstart', that.hide.bind(that));
      } else {
        divElem.addEventListener('click', emitMenuSelect.bind(that, children[i]));
        divElem.addEventListener('touchstart', emitMenuSelect.bind(that, children[i]));
        divElem.className = 'default-interactive-marker-menu-entry';
      }
    }

  }

  // construct DOM element
  makeUl(this.menuDomElem, allMenus[0]);
};

/**
 * Shoe the menu DOM element.
 *
 * @param control - the control for the menu
 * @param event - the event that caused this
 */
ROS3D.InteractiveMarkerMenu.prototype.show = function(control, event) {
  if (event && event.preventDefault) {
    event.preventDefault();
  }

  this.controlName = control.name;

  // position it on the click
  if (event.domEvent.changedTouches !== undefined) {
    // touch click
    this.menuDomElem.style.left = event.domEvent.changedTouches[0].pageX + 'px';
    this.menuDomElem.style.top = event.domEvent.changedTouches[0].pageY + 'px';
  } else {
    // mouse click
    this.menuDomElem.style.left = event.domEvent.clientX + 'px';
    this.menuDomElem.style.top = event.domEvent.clientY + 'px';
  }
  document.body.appendChild(this.overlayDomElem);
  document.body.appendChild(this.menuDomElem);
};

/**
 * Hide the menu DOM element.
 *
 * @param event (optional) - the event that caused this
 */
ROS3D.InteractiveMarkerMenu.prototype.hide = function(event) {
  if (event && event.preventDefault) {
    event.preventDefault();
  }

  document.body.removeChild(this.overlayDomElem);
  document.body.removeChild(this.menuDomElem);
};

Object.assign(ROS3D.InteractiveMarkerMenu.prototype, THREE.EventDispatcher.prototype);
