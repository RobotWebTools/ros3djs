import Vue from 'vue'
import Router from 'vue-router'
import Markers from '@/components/Markers'

Vue.use(Router)

export default new Router({
  routes: [
    {
      path: '/markers',
      name: 'Markers',
      component: Markers
    }
  ]
})
