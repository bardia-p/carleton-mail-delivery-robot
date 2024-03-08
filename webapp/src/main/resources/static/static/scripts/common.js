/**
 * JavaScript file that defines all the toasts used.
 */

// toast configurations
toastr.options = {
    "closeButton": true,
    "debug": false,
    "newestOnTop": false,
    "progressBar": true,
    "positionClass": "toast-top-right",
    "preventDuplicates": false,
    "onclick": null,
    "showDuration": "200",
    "hideDuration": "500",
    "timeOut": "2000",
    "extendedTimeOut": "1000",
    "showEasing": "swing",
    "hideEasing": "linear",
    "showMethod": "fadeIn",
    "hideMethod": "fadeOut"
};

/**
 *
 * @param {"success" | "info" | "warning" | "error"} severity
 * @param {string} title
 * @param {string} message
 * @param {boolean} redirect
 *
 */
const setToast = (severity, title, message, redirect= false) => {
    if (!redirect) {
        toastr[severity](message, title);
        return;
    }
    // add the toast to the session to be displayed upon redirect
    let toasts = sessionStorage.getItem("toasts");
    if (!toasts) {
        toasts = [];
    } else {
        toasts = JSON.parse(toasts);
    }
    toasts.push({ severity, title, message });
    sessionStorage.setItem("toasts", JSON.stringify(toasts));
}

$(document).ready(() => {
    // on document load, see if there are messages that needs to be showed
    let toasts = sessionStorage.getItem("toasts");
    if (toasts) {
        toasts = JSON.parse(toasts);
        for (const toast of toasts) {
            toastr[toast.severity](toast.message, toast.title);
        }
        toasts = [];
        sessionStorage.setItem("toasts", JSON.stringify(toasts));
    }
});