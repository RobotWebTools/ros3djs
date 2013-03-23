ROS3D.InteractiveMarkerMenu = function(menuEntries) {
  console.log(menuEntries);
  var allMenus = [];
  allMenus[0] = {
    children : []
  };

  THREE.EventDispatcher.call(this);

  var that = this;

  this.menuDomElem = document.createElement("div");
  this.menuDomElem.style.position = "absolute";
  this.menuDomElem.className = "interactive_marker_menu";
  this.menuDomElem.addEventListener("contextmenu", function(event) {
    event.preventDefault();
  });

  this.overlayDomElem = document.createElement("div");
  this.overlayDomElem.className = "interactive_marker_overlay";

  this.hideListener = this.hide.bind(this);
  this.overlayDomElem.addEventListener("contextmenu", this.hideListener);
  this.overlayDomElem.addEventListener("click", this.hideListener);

  // parse all entries
  for ( var i = 0; i < menuEntries.length; i++) {
    var entry = menuEntries[i];
    var id = entry.id;
    allMenus[id] = {
      title : entry.title,
      id : id,
      children : []
    };
  }

  // link children to parents
  for ( var i = 0; i < menuEntries.length; i++) {
    var entry = menuEntries[i];
    var id = entry.id;
    var menu = allMenus[id];
    var parent = allMenus[entry.parent_id];
    parent.children.push(menu);
  }

  function emitMenuSelect(menuEntry, domEvent) {
    this.dispatchEvent({
      type : "menu_select",
      domEvent : domEvent,
      id : menuEntry.id,
      controlName : this.controlName
    });
    this.hide(domEvent);
  }

  // create html menu, starting from root (id 0)
  function makeUl(parentDomElem, parentMenu) {

    var ulElem = document.createElement("ul");
    parentDomElem.appendChild(ulElem);

    var children = parentMenu.children;

    for ( var i = 0; i < children.length; i++) {
      var liElem = document.createElement("li");
      var divElem = document.createElement("div");
      divElem.appendChild(document.createTextNode(children[i].title));
      ulElem.appendChild(liElem);
      liElem.appendChild(divElem);

      if (children[i].children.length > 0) {
        makeUl(liElem, children[i]);
        divElem.addEventListener("click", that.hide.bind(that));
      } else {
        divElem.addEventListener("click", emitMenuSelect.bind(that, children[i]));
        divElem.className = "interactive_marker_menuentry";
      }
    }

  }

  // construct dom element
  makeUl(this.menuDomElem, allMenus[0]);
};

ROS3D.InteractiveMarkerMenu.prototype.show = function(control, event) {
  if (event && event.preventDefault) {
    event.preventDefault();
  }

  this.controlName = control.name;

  // this.overlayDomElem.style.visibility = "visible";
  this.menuDomElem.style.left = event.domEvent.clientX + 'px';
  this.menuDomElem.style.top = event.domEvent.clientY + 'px';
  document.body.appendChild(this.overlayDomElem);
  document.body.appendChild(this.menuDomElem);
};

ROS3D.InteractiveMarkerMenu.prototype.hide = function(event) {
  if (event && event.preventDefault) {
    event.preventDefault();
  }

  document.body.removeChild(this.overlayDomElem);
  document.body.removeChild(this.menuDomElem);
};
