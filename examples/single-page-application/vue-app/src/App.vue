<template>
  <div id="app">
    <p>This is an app meant to demonstrate how to use the ros3djs module in a Vue app</p>
    <p>Examples:</p>
    <ul>
      <li><router-link to="markers">Markers</router-link></li>
    </ul>

    <div id="examples">
      <router-view/>
    </div>
  </div>
</template>

<script>
export default {
  name: 'App',
}
</script>

<style>
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: left;
  color: #2c3e50;
  margin: 1em;
}

li {
  list-style-type: none;
}

#examples {
  box-shadow:
    rgba(10, 16, 20, 0.24) 0px 2px 2px,
    rgba(10, 16, 20, 0.12) 0px 0px 2px;
  padding: 1em;
}
</style>
